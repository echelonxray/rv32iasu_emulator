#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <poll.h>
#include <termios.h>
#include <signal.h>
#include <semaphore.h>
#include <unistd.h>
#include <limits.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/ioctl.h>

#define STDIN 0
#define STDOUT 1
#define STDERR 2

#define DIR_UP 0
#define DIR_DOWN 1
#define DIR_LEFT 2
#define DIR_RIGHT 3

#define USIG_PAUSE (SIGRTMIN + 0)
#define USIG_P_ACK (SIGRTMIN + 1)

// START: Build-Time Configuration Definitions

// What direction should the snake be pointing at start?
#define STARTING_DIRECTION DIR_UP
// How long should the snake be at the start?  This must be at least 2.
#define STARTING_LENGTH 5
// How many cells should be added to the snake when food is consumed?
// If GROW_BY_INCREMENT is not 0, this will change after the first unit 
// of food is consumed.
#define STARTING_GROW_BY 2
// What should be added to the "Grow By" rate after food is consumed?
#define GROW_BY_INCREMENT 2
// How long should the delay between ticks be in milliseconds?
#define DELAY_TIME_MS 150

// END: Build-Time Configuration Definitions

unsigned int score;
unsigned int not_paused;
unsigned int curr_term_width;
unsigned int curr_term_height;
unsigned int term_width;
unsigned int term_height;
unsigned int grid_width;
unsigned int grid_height;
sem_t sem0;
sem_t sem1;

struct Snake {
  unsigned int new_direction;
  unsigned int direction;
  unsigned int length;
  unsigned int grid_used_length;
  unsigned int grow_by;
  struct GridCell *cells;
};

struct GridCell {
  signed int x;
  signed int y;
};

struct ThreadInfo {
  struct Snake *snake;
  struct GridCell *food;
  char *display_content;
};

void sem_wai2(sem_t *sem) {
  while (sem_wait(sem) == -1) {
    if (errno != EINTR) {
      return;
    }
  }
  return;
}

void signal_handle(signed int sig_number) {
  // Signal Handler
  int errno_backup = errno; // Save errno from interrupted thread context.
  
  if        (sig_number == SIGWINCH) {
    // Running in execution context of child thread: Signal Listener
    // Terminal Resize Event
    
    pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, NULL);
    sem_wai2(&sem1);
    
    // Get the terminal size
    struct winsize term_size;
    ioctl(STDOUT, TIOCGWINSZ, &term_size);
    // TODO: Consider checking ioctl return value
    curr_term_width = term_size.ws_col;
    curr_term_height = term_size.ws_row;
    
    if (term_width != curr_term_width || term_height != curr_term_height) {
      // Pause the game
      
      // Nothing to pause if already paused
      if (not_paused) {
        kill(0, USIG_PAUSE); // Dispatch Pause signal to Game Loop thread
        
        sigset_t wait_signal;
        sigemptyset(&wait_signal);
        sigaddset(&wait_signal, USIG_P_ACK);
        sigwaitinfo(&wait_signal, NULL);
        
        not_paused = 0;
      }
    }
    
    // Clear the terminal
    // Clear the scrollback buffer
    // Reset the terminal cursor position to the top left
    dprintf(STDOUT, "\e[1;1H\e[0J\e[2J\e[3J\e[1;1H");
    
    dprintf(STDOUT, "Game Paused\n\r");
    dprintf(STDOUT, "Press E to unpause\n\r");
    dprintf(STDOUT, "Press Q to quit\n\r");
    dprintf(STDOUT, "Press M to leave the current game and return to the menu (Not Implemented)\n\r");
    dprintf(STDOUT, "Current Score: %d\n\r", score);
    dprintf(STDOUT, "Expected terminal size for current game: %dx%d\n\r", term_width, term_height);
    dprintf(STDOUT, "Current terminal size: %dx%d\n\r", curr_term_width, curr_term_height);
    dprintf(STDOUT, "The terminal size must match the expected size before unpause will be allowed.\n\r");
    
    sem_post(&sem1);
    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
  } else if (sig_number == USIG_PAUSE) {
    // Running in execution context of child thread: Game Loop
    // Pause Event
    // The entirety of this execution pathway from function call to return 
    //   occurs in a state of enabled pthread thread cancellability.
    
    // Inform the other thread that pause has been received 
    // and is now being handled.  The other thread can now 
    // take over display drawing until unpaused.
    kill(0, USIG_P_ACK);
    
    // Wait for unpause (Another USIG_PAUSE signal)
    sigset_t wait_signal;
    sigemptyset(&wait_signal);
    sigaddset(&wait_signal, USIG_PAUSE);
    sigwaitinfo(&wait_signal, NULL);
  }
  
  errno = errno_backup; // Restore errno from interrupted thread context.
  return;
}

signed int gen_random_number(signed int min, signed int max) {
  // Find a random integer somewhere from [min] through [max]
  
  if (min > max) {
    return 0;
  }
  unsigned int length = (max - min) + 1;
  return min + (rand() % length);
}

void rand_food_location(struct GridCell *food, struct Snake *snake, unsigned int width, unsigned int height) {
  // Generate a new random food location and assign it to the struct pointed at by [food]
  
  // How many Grid spaces are there?
  unsigned int the_grid_space = (width * height);
  // Subtract the Grid spaces used by the used snake length to get a count of spaces empty
  the_grid_space -= snake->grid_used_length;
  // Find a random Grid index within those spaces that are empty
  the_grid_space = gen_random_number(0, the_grid_space - 1);
  
  // Walk through all Grid spaces from index 0 to the space chosen at random.
  // For each space that is occupied by a snake cell, increment the index of 
  // the chosen space. This has the effect of translating the index of an empty 
  // Grid space of the empty Grid spaces, to an index of an empty Grid 
  // space within the entirety of all of the Grid spaces.
  // 
  // Example: 
  // Suppose that we chose index: 1 (The 2nd empty Grid space).
  // This is the status of the beginning of the first line of the 
  // grid ('+' is a snake cell and '_' is an empty space): "_+_" .
  //                                              Indexes:  012
  // 
  // In this case, this procedure should convert the index: 1 to the 
  // index: 2 (The index on the Grid of the 2nd empty Grid space).
  unsigned int i = 0;
  for (unsigned int y = 0; y < height; y++) {
    for (unsigned int x = 0; x < width; x++) {
      // Iterate through the snake to check if any cells occupy the space
      for (unsigned int j = 0; j < snake->length; j++) {
        if (snake->cells[j].x == (signed int)x && snake->cells[j].y == (signed int)y) {
          the_grid_space++;
          // Break the loop because it is possible for a snake to have 
          // several cells on the same x, y coordinate.  However, we only 
          // want to count one overlap per x, y location.
          // 
          // Since we have incremented the chosen index, it is not possible for us 
          // to have reached the chosen index yet. Therefore, we can skip that check 
          // on this pass.
          goto continue_grid_loop;
        }
      }
      
      // Have we reached the randomly chosen empty grid space?
      if (i == the_grid_space) {
        // Derive the x and y coordinates of that space on the grid and 
        // assign them to the food.  
        food->x = the_grid_space % width;
        food->y = the_grid_space / width;
        // We have finished generating a new food location.
        return;
      }
      
      continue_grid_loop:
      i++;
    }
  }
  
  // Should be unreachable
  food->x = -1;
  food->y = -1;
  return;
}

void regen_buffer(char *buffer, struct Snake *snake, struct GridCell *food) {
  // Render the Grid into the Buffer
  
  // Render Line 1 with the Score Count
  {
    char format_string[24];
    snprintf(format_string, 24, "Score: %%-%dd", term_width - 7);
    snprintf(buffer, term_width + 1, format_string, score);
    buffer += strlen(buffer);
#ifndef NOEXPLICITNEWLINES
    *buffer = '\n';
    buffer++;
    *buffer = '\r';
    buffer++;
#endif
  }
  
  *buffer = 0xE2;
  buffer++;
  *buffer = 0x94;
  buffer++;
  *buffer = 0x8F;
  buffer++;
  for (unsigned int x = 0; x < grid_width; x++) {
    *buffer = 0xE2;
    buffer++;
    *buffer = 0x94;
    buffer++;
    *buffer = 0x81;
    buffer++;
  }
  *buffer = 0xE2;
  buffer++;
  *buffer = 0x94;
  buffer++;
  *buffer = 0x93;
  buffer++;
#ifndef NOEXPLICITNEWLINES
  *buffer = '\n';
  buffer++;
  *buffer = '\r';
  buffer++;
#endif
  
  unsigned int snake_length = snake->length;
  
  for (unsigned int y = 0; y < grid_height; y++) {
    
    *buffer = 0xE2;
    buffer++;
    *buffer = 0x94;
    buffer++;
    *buffer = 0x83;
    buffer++;
    
    for (unsigned int x = 0; x < grid_width; x++) {
      // Is this a Snake Cell?
      for (unsigned int i = 0; i < snake_length; i++) {
        if (snake->cells[i].x == (signed int)x && snake->cells[i].y == (signed int)y) {
          // Regen Snake Cell
          *buffer = '+';
          buffer++;
          goto next_grid_cell;
        }
      }
      
      // Is this a Food Cell?
      if (food->x == (signed int)x && food->y == (signed int)y) {
        // Regen Food Cell
        *buffer = 'x';
        buffer++;
        goto next_grid_cell;
      }
      
      // If none of the above, it must be an Empty Cell
      // Regen Empty Cell
      *buffer = ' ';
      buffer++;
      
      next_grid_cell:;
    }
    
    *buffer = 0xE2;
    buffer++;
    *buffer = 0x94;
    buffer++;
    *buffer = 0x83;
    buffer++;
    
#ifndef NOEXPLICITNEWLINES
    // In case we are running on a TTY that does not get up-to-date terminal size 
    // information-such as over a COM port-explicitly output a New Line and Carriage Return.  
    // This will keep the display sane in case the terminal is wider than the TTY believes.  
    // It prevents reliance on wrapping for new lines.
    // 
    // This can be disabled by setting the above preprocessor definition, 'NOEXPLICITNEWLINES'
    *buffer = '\n';
    buffer++;
    *buffer = '\r';
    buffer++;
#endif
    
  }
  
  *buffer = 0xE2;
  buffer++;
  *buffer = 0x94;
  buffer++;
  *buffer = 0x97;
  buffer++;
  for (unsigned int x = 0; x < grid_width; x++) {
    *buffer = 0xE2;
    buffer++;
    *buffer = 0x94;
    buffer++;
    *buffer = 0x81;
    buffer++;
  }
  *buffer = 0xE2;
  buffer++;
  *buffer = 0x94;
  buffer++;
  *buffer = 0x9B;
  buffer++;
  
  // Make sure the string is NULL terminated
  *buffer = 0;
  
  return;
}

void snake_append_cells(struct Snake *snake, unsigned int num_to_add) {
  unsigned int i = snake->length;
  unsigned int last_cell_index = i - 1;
  snake->length += num_to_add;
  snake->cells = realloc(snake->cells, snake->length * sizeof(struct GridCell));
  // TODO: Handle realloc failure
  // TODO: Consider changing to reallocarray() to easily safely handle multiplication overflow
  while (i < snake->length) {
    snake->cells[i] = snake->cells[last_cell_index];
    i++;
  }
  return;
}

void snake_crawl(struct Snake *snake, struct GridCell *food) {
  // Crawl Snake Forward
  
  unsigned int width = grid_width;
  unsigned int height = grid_height;
  
  signed int prev_cell_x = snake->cells[0].x;
  signed int prev_cell_y = snake->cells[0].y;
  signed int head_cell_x = prev_cell_x;
  signed int head_cell_y = prev_cell_y;
  
  // Update the direction
  snake->direction = snake->new_direction;
  
  // Move the head in the direction of the snake
  if        (snake->direction == DIR_UP) {
    head_cell_y -= 1;
  } else if (snake->direction == DIR_DOWN) {
    head_cell_y += 1;
  } else if (snake->direction == DIR_LEFT) {
    head_cell_x -= 1;
  } else {                    // DIR_RIGHT
    head_cell_x += 1;
  }
  
  // Handle Wrapping
  if        (head_cell_x < 0) {
    head_cell_x += width;
  } else if (head_cell_x >= (signed int)width) {
    head_cell_x -= width;
  } else if (head_cell_y < 0) {
    head_cell_y += height;
  } else if (head_cell_y >= (signed int)height) {
    head_cell_y -= height;
  }
  
  // Are we still expanding from cells added to the snake?
  // This should be handled before checking for food consumption 
  // because snake->length might be increased there.  This crawl
  // tick does not apply to cells added during it.  Cells added 
  // during this tick will be expanded in subsequent ticks.
  if (snake->grid_used_length < snake->length) {
    snake->grid_used_length++;
  }
  
  // Did we consume food?
  if (head_cell_x == food->x && head_cell_y == food->y) {
    // Handle food consume
    score += 1;
    snake_append_cells(snake, snake->grow_by);
    snake->grow_by += GROW_BY_INCREMENT;
    rand_food_location(food, snake, width, height);
  }
  
  snake->cells[0].x = head_cell_x;
  snake->cells[0].y = head_cell_y;
  
  signed int old_cell_x;
  signed int old_cell_y;
  
  old_cell_x = snake->cells[1].x;
  old_cell_y = snake->cells[1].y;
  snake->cells[1].x = prev_cell_x;
  snake->cells[1].y = prev_cell_y;
  prev_cell_x = old_cell_x;
  prev_cell_y = old_cell_y;
  
  // Move along the rest of the snake body
  for (unsigned int i = 2; i < snake->length; i++) {
    if (head_cell_x == prev_cell_x && head_cell_y == prev_cell_y) {
      // TODO: Handle body collision
    }
    
    old_cell_x = snake->cells[i].x;
    old_cell_y = snake->cells[i].y;
    snake->cells[i].x = prev_cell_x;
    snake->cells[i].y = prev_cell_y;
    prev_cell_x = old_cell_x;
    prev_cell_y = old_cell_y;
  }
  
  return;
}

void* game_loop(void *thread_info) {
  pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, NULL);
  
  struct timespec target_sleep_time;
  target_sleep_time.tv_sec = DELAY_TIME_MS / 1000;
  target_sleep_time.tv_nsec = (DELAY_TIME_MS % 1000) * 1000000;
  
  struct ThreadInfo *th_info = (struct ThreadInfo*)thread_info;
  struct Snake *snake = th_info->snake;
  struct GridCell *food = th_info->food;
  char *display_content = th_info->display_content;
  
  sigset_t pause_signal;
  sigemptyset(&pause_signal);
  sigaddset(&pause_signal, USIG_PAUSE);
  
  // Game Loop
  while (1) {
    // Get processing start time
    struct timeval loop_start_time;
    gettimeofday(&loop_start_time, NULL);
    
    // The meat of the game loop:
    // --Get a lock on the thread synchronization tool (Mutex or Binary Semaphore)
    // --Crawl the snake forward
    // --Use the newly generated data to renderer the display buffer
    // --Print/Draw the display buffer
    // --Release the lock
    sem_wai2(&sem0);
    snake_crawl(snake, food);
    regen_buffer(display_content, snake, food);
    dprintf(STDOUT, "\e[%d;%dH%s", 1, 1, display_content);
    sem_post(&sem0);
    
    // The rest of the loop below calculates sleep time and sleeps while 
    // thread cancellation is enabled, providing a clean exit point.
    
    // Get processing end time
    struct timeval loop_end_time;
    gettimeofday(&loop_end_time, NULL);
    
    // Thread sleep calculations, nanosleep(), interruptibility, and cancellability
    {
      pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
      // Allow thread interruptions for Game Pausing
      // This thread must only be interruptible while cancellation is 
      //   enabled to prevent it from hanging on sigwaitinfo() during a 
      //   pause-quit event outside of nanosleep().q
      pthread_sigmask(SIG_UNBLOCK, &pause_signal, NULL);
      
      // How much time elapsed while processing?
      if (loop_end_time.tv_usec < loop_start_time.tv_usec) {
        loop_end_time.tv_sec--;
        loop_end_time.tv_usec += (1000000 - loop_start_time.tv_usec);
      } else {
        loop_end_time.tv_usec -= loop_start_time.tv_usec;
      }
      loop_end_time.tv_sec -= loop_start_time.tv_sec;
      
      // Init nanosleep struct with time 0 in case processing took too long.
      // nanosleep() much still be called to give the thread an opportunity to be cancelled.
      struct timespec actual_sleep_time;
      actual_sleep_time.tv_sec = 0;
      actual_sleep_time.tv_nsec = 0;
      
      // Is the elapsed processing time less than the target sleep time?
      if (((loop_end_time.tv_sec == target_sleep_time.tv_sec) && (((long)loop_end_time.tv_usec * 1000) < target_sleep_time.tv_nsec)) || \
           (loop_end_time.tv_sec <  target_sleep_time.tv_sec)) {
        
        // Set the nanosleep time to the target time remaining
        actual_sleep_time.tv_sec = target_sleep_time.tv_sec - loop_end_time.tv_sec;
        if ((long)loop_end_time.tv_usec * 1000 > target_sleep_time.tv_nsec) {
          actual_sleep_time.tv_sec--;
          actual_sleep_time.tv_nsec = (1000000l - loop_end_time.tv_usec) * 1000 + target_sleep_time.tv_nsec;
        } else {
          actual_sleep_time.tv_nsec = target_sleep_time.tv_nsec - ((long)loop_end_time.tv_usec * 1000);
        }
        actual_sleep_time.tv_nsec = target_sleep_time.tv_nsec - (loop_end_time.tv_usec * 1000);
        
      }
      
      struct timespec rem_sleep_time;
      while (nanosleep(&actual_sleep_time, &rem_sleep_time) == -1) {
        if (errno != EINTR) {
          break;
        }
        actual_sleep_time = rem_sleep_time;
      }
      
      // Block thread interruptions for Game Pausing
      pthread_sigmask(SIG_BLOCK, &pause_signal, NULL);
      pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, NULL);
    }
  }
  return NULL;
}

void* signal_receiver_thread(void *arg) {
  struct timespec sleep_time;
  sleep_time.tv_sec = 10;
  sleep_time.tv_nsec = 0;
  
  {
    sigset_t del_signal_mask;
    sigemptyset(&del_signal_mask);
    sigaddset(&del_signal_mask, SIGWINCH);
    pthread_sigmask(SIG_UNBLOCK, &del_signal_mask, NULL);
  }
  
  while (1) {
    nanosleep(&sleep_time, NULL);
  }
  
  return NULL;
}

signed int main(signed int argc, char *argv[], char *envp[]) {
  
  // Seed the PRNG
  {
    time_t s = time(NULL);
    if (s == (time_t)-1) {
      exit(2);
    }
    srand((unsigned int)(s % INT_MAX));
  }
  
  // Mask Signals: SIGWINCH, USIG_PAUSE, USIG_P_ACK
  sigset_t add_signal_mask;
  {
    sigemptyset(&add_signal_mask);
    sigaddset(&add_signal_mask, SIGWINCH);
    sigaddset(&add_signal_mask, USIG_PAUSE);
    sigaddset(&add_signal_mask, USIG_P_ACK);
    sigprocmask(SIG_BLOCK, &add_signal_mask, NULL);
  }
  
  // Setup the Signal Handler for Terminal Resize Events (SIGWINCH)
  {
    struct sigaction sig_action;
    sig_action.sa_handler = &signal_handle;
    sigemptyset(&(sig_action.sa_mask));
    sig_action.sa_flags = 0;
    sigaction(SIGWINCH, &sig_action, NULL);
  }
  
  // Setup the Signal Handler for Pause Notifications (USIG_PAUSE)
  {
    struct sigaction sig_action;
    sig_action.sa_handler = &signal_handle;
    sigemptyset(&(sig_action.sa_mask));
    sig_action.sa_flags = 0;
    sigaction(USIG_PAUSE, &sig_action, NULL);
  }
  
  // Determine the terminal size
  {
    // Get the terminal size
    struct winsize term_size;
    ioctl(STDOUT, TIOCGWINSZ, &term_size);
    // TODO: Consider checking ioctl return value
    curr_term_width = term_size.ws_col;
    curr_term_height = term_size.ws_row;
    term_width = curr_term_width;
    term_height = curr_term_height;
    grid_width = term_width - 2;
    grid_height = term_height - 3;
  }
  
  // Allocated the memory for the Display Buffer
  char* display_content = 0;
  {
    unsigned int buffer_size = (term_width + 1) * term_height * sizeof(char) * 4;
    display_content = malloc(buffer_size);
    // TODO: Handle malloc failure
  }
  
  // Init the Snake
  struct Snake snake;
  {
    snake.new_direction = STARTING_DIRECTION;
    snake.direction = STARTING_DIRECTION;
    snake.length = STARTING_LENGTH;
    snake.grid_used_length = STARTING_LENGTH;
    snake.grow_by = STARTING_GROW_BY;
    snake.cells = malloc(sizeof(struct GridCell) * STARTING_LENGTH);
    // TODO: Handle malloc failure
    snake.cells[0].x = grid_width / 2;
    snake.cells[0].y = grid_height / 2;
    for (unsigned int i = 1; i < STARTING_LENGTH; i++) {
      if (STARTING_DIRECTION == DIR_UP) {
        snake.cells[i].x = snake.cells[0].x;
        snake.cells[i].y = snake.cells[0].y + i;
      } else if (STARTING_DIRECTION == DIR_DOWN) {
        snake.cells[i].x = snake.cells[0].x;
        snake.cells[i].y = snake.cells[0].y - i;
      } else if (STARTING_DIRECTION == DIR_LEFT) {
        snake.cells[i].x = snake.cells[0].x + i;
        snake.cells[i].y = snake.cells[0].y;
      } else {
        snake.cells[i].x = snake.cells[0].x - i;
        snake.cells[i].y = snake.cells[0].y;
      }
    }
  }
  
  // Init the Food
  struct GridCell food;
  rand_food_location(&food, &snake, grid_width, grid_height);
  
  // START: Setup the Terminal
  // Set TTY to Raw mode
  struct termios old_tty_settings;
  {
    struct termios raw_tty_settings;
    signed int retval;
    retval = ioctl(STDOUT, TCGETS, &old_tty_settings);
    if (retval == -1) {
      exit(25);
    }
    memcpy(&raw_tty_settings, &old_tty_settings, sizeof(struct termios));
    raw_tty_settings.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    raw_tty_settings.c_oflag &= ~OPOST;
    raw_tty_settings.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    raw_tty_settings.c_cflag &= ~(CSIZE | PARENB);
    raw_tty_settings.c_cflag |= CS8;
    retval = ioctl(STDOUT, TCSETS, &raw_tty_settings);
    if (retval == -1) {
      exit(26);
    }
  }
  
  // Disable the cursor
  dprintf(STDOUT, "\e[?25l");
  
  // Clear the terminal
  // Clear the scrollback buffer
  // Reset the terminal cursor position to the top left
  dprintf(STDOUT, "\e[1;1H\e[0J\e[2J\e[3J\e[1;1H");
  // END: Setup the Terminal
  
  // Init polling struct
  struct pollfd pfd;
  pfd.fd = STDIN;
  pfd.events = POLLIN;
  
  not_paused = 1;
  score = 0;
  
  // Create a New Thread for the Game Loop
  pthread_t pthread_id_gameloop;
  pthread_t pthread_id_signals;
  struct ThreadInfo th_info;
  {
    th_info.snake = &snake;
    th_info.food = &food;
    th_info.display_content = display_content;
    sem_init(&sem0, 0, 1);
    sem_init(&sem1, 0, 1);
    if (pthread_create(&pthread_id_gameloop, NULL, &game_loop, (void*)&th_info) != 0) {
      exit(10);
    }
    if (pthread_create(&pthread_id_signals, NULL, &signal_receiver_thread, NULL) != 0) {
      exit(10);
    }
  }
  
  unsigned int exit_code = 0;
  // Main Event Loop
  while (1) {
    // Listen for key-presses (Data) on STDIN
    signed int retval = poll(&pfd, 1, 3000);
    if (retval > 0) {
      if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
        // Unhanded or Fault condition
        exit_code = 1;
        break;
      } else {
        // POLLIN: Data received on STDIN
        char data = 0;
        read(STDIN, &data, 1);
        if        (data == 'q' || data == 'Q') {
          break;
        } else if (data == 'e' || data == 'E') {
          sem_wai2(&sem1);
          if (term_width == curr_term_width && term_height == curr_term_height) {
            kill(0, USIG_PAUSE); // Dispatch Pause signal to Game Loop thread
            if (not_paused) {
              sigset_t wait_signal;
              sigemptyset(&wait_signal);
              sigaddset(&wait_signal, USIG_P_ACK);
              sigwaitinfo(&wait_signal, NULL);
              
              not_paused = 0;
              
              // Clear the terminal
              // Clear the scrollback buffer
              // Reset the terminal cursor position to the top left
              dprintf(STDOUT, "\e[1;1H\e[0J\e[2J\e[3J\e[1;1H");
              
              dprintf(STDOUT, "Game Paused\n\r");
              dprintf(STDOUT, "Press E to unpause\n\r");
              dprintf(STDOUT, "Press Q to quit\n\r");
              dprintf(STDOUT, "Press M to leave the current game and return to the menu (Not Implemented)\n\r");
              dprintf(STDOUT, "Current Score: %d\n\r", score);
              dprintf(STDOUT, "Expected terminal size for current game: %dx%d\n\r", term_width, term_height);
              dprintf(STDOUT, "Current terminal size: %dx%d\n\r", curr_term_width, curr_term_height);
              dprintf(STDOUT, "The terminal size must match the expected size before unpause will be allowed.\n\r");
            } else {
              not_paused = 1;
            }
          }
          sem_post(&sem1);
        } else {
          // Game input should not be accepted if the game is paused
          if (not_paused) {
            if (data == 'w' || data == 'W') {
              sem_wai2(&sem0);
              if (snake.direction == DIR_UP || snake.direction == DIR_LEFT || snake.direction == DIR_RIGHT) {
                snake.new_direction = DIR_UP;
              }
              sem_post(&sem0);
            } else if (data == 's' || data == 'S') {
              sem_wai2(&sem0);
              if (snake.direction == DIR_DOWN || snake.direction == DIR_LEFT || snake.direction == DIR_RIGHT) {
                snake.new_direction = DIR_DOWN;
              }
              sem_post(&sem0);
            } else if (data == 'a' || data == 'A') {
              sem_wai2(&sem0);
              if (snake.direction == DIR_UP || snake.direction == DIR_LEFT || snake.direction == DIR_DOWN) {
                snake.new_direction = DIR_LEFT;
              }
              sem_post(&sem0);
            } else if (data == 'd' || data == 'D') {
              sem_wai2(&sem0);
              if (snake.direction == DIR_UP || snake.direction == DIR_RIGHT || snake.direction == DIR_DOWN) {
                snake.new_direction = DIR_RIGHT;
              }
              sem_post(&sem0);
            }
          }
        }
      }
    } else if (retval < 0) {
      // Poll syscall error
      // All errors treated as fatal except for 
      // EINTR (Signal received while polling) 
      // In this case, simply restart polling.
      if (errno != EINTR) {
        exit_code = 2;
        break;
      }
    }
  }
  
  // Clean up and exit:
  
  // Join the threads here
  pthread_cancel(pthread_id_gameloop);
  pthread_cancel(pthread_id_signals);
  pthread_join(pthread_id_gameloop, NULL);
  pthread_join(pthread_id_signals, NULL);
  sem_destroy(&sem0);
  sem_destroy(&sem1);
  
  // START: Restore the Terminal
  // Enable the cursor
  dprintf(STDOUT, "\e[?25h");
  
  // Restore the TTY to the original mode
  ioctl(STDOUT, TCSETS, &old_tty_settings);
  // END: Restore the Terminal
  
  // Free the memory
  free(snake.cells);
  free(display_content);
  
  dprintf(STDOUT, "\n");
  
  if (exit_code > 0) {
    exit_code += 50;
  }
  
  return exit_code;
}
