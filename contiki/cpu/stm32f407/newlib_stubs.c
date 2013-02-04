/*
 * newlib_stubs.c
 *
 *  Created on: 31 Jan 2013
 *      Author: Philipp Spliethoff 
 *        Mail: philipp.spliethoff@udo.edu
 *
 * This File implements systemcalls used by the Newlib C-Library. For
 * further information look at:
 * http://sourceware.org/newlib/libc.html#Syscalls
 *
 */
#include <errno.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <sys/unistd.h>

#ifdef USE_USB
extern CDC_IF_Prop_TypeDef  APP_FOPS;
#endif /* USE_USB */

#ifndef STDOUT_LINE
#warning no STDOUT_LINE specified. Defaulting to USART3.
#define STDOUT_LINE 3
#endif

#ifndef STDERR_LINE
#warning no STDERR_LINE specified. Defaulting to USART3.
#define STDERR_LINE 3
#endif

#ifndef STDIN_LINE
#warning no STDIN_LINE specified. Defaulting to USART3.
#define STDIN_LINE 3
#endif

#undef errno
extern int errno;


/*
 * environ
 * A pointer to a list of environment variables and their values. 
 * For a minimal environment, this empty list is adequate:
 */
char *__env[1] = { 0 };
char **environ = __env;

/*
 *  Exit a program without cleaning up files. If your system doesn't
 *  provide this, it is best to avoid linking with subroutines that
 *  require it (exit, system).
 */
void _exit(int status) {
}

/*
 * Close a File.
 */
int _close(int file) {
  return -1;
}

/*
 * Transfer control to a new process.
 * 
 */
int _execve(char *name, char **argv, char **env) {
  errno = ENOMEM;
  return -1;
}


/*
 * Create a new process.
 */
int _fork() {
  errno = EAGAIN;
  return -1;
}

/*
 * Status of an open file. For consistency with other minimal
 * implementations in these examples, all files are regarded as
 * character special devices.  The `sys/stat.h' header file required
 * is distributed in the `include' subdirectory for this C library.
 */
int _fstat(int file, struct stat *st) {
  st->st_mode = S_IFCHR;
  return 0;
}

/*
 * Process-ID; this is sometimes used to generate strings unlikely to
 * conflict with other processes. Minimal implementation, for a system
 * without processes:
 */
int _getpid() {
  return 1;
}

/*
 * Query whether output stream is a terminal. 
 *
 * If file is one of STDOUT STDIN or STDERR we are on a tty. Return
 * 1. otherwise return a bad filedescriptor error.
 */
int _isatty(int file) {
  switch (file){
  case STDOUT_FILENO:
  case STDERR_FILENO:
  case STDIN_FILENO:
    return 1;
  default:
    errno = EBADF;
    return 0;
  }
}

/*
 * Send Kill_signal to a process. Not implemented
 */
int _kill(int pid, int sig) {
    errno = EINVAL;
    return (-1);
}

/*
 * Establish a new name for an existing file. 
 */
int _link(char *old, char *new) {
    errno = EMLINK;
    return -1;
}

/*
 *  Set position in a file. Minimal implementation:
 */
int _lseek(int file, int ptr, int dir) {
    return 0;
}

/*
 sbrk
 Increase program data space.
 Malloc and related functions depend on this
 */
caddr_t _sbrk(int incr) {

  /*    extern char _ebss; // Defined by the linker
    static char *heap_end;
    char *prev_heap_end;

    if (heap_end == 0) {
        heap_end = &_ebss;
    }
    prev_heap_end = heap_end;

char * stack = (char*) __get_MSP();
     if (heap_end + incr >  stack)
     {
         _write (STDERR_FILENO, "Heap and stack collision\n", 25);
         errno = ENOMEM;
         return  (caddr_t) -1;
         //abort ();
     }

    heap_end += incr;
    return (caddr_t) prev_heap_end;
  */
  return 0;
}

/*
 * Read a character from a file. `libc' subroutines will use this system
 * routine for input from all files, including stdin Returns -1 on
 * error or blocks until the number of characters have been read.
 * 
 * not implemented
 */
int _read(int file, char *ptr, int len) {
  /* int i; */
/*   switch (file) { */
/*   case STDIN_FILENO: */
/*     for (i=0; i<len; i++) */
/* #if   STDIN_LINE == 3 /\* USART3 *\/ */
/*       usart3_writeb((unsigned char)ptr[i]); */
/*     /\* call uart3 send buffer function *\/ */
/* #warning TODO implement USART3 send buffer function */

/* #elif STDIN_LINE == 4 /\* UART4 *\/ */
/*     uart4_writeb((unsigned char)ptr[i]); */
/*     #warning TODO implement UART4 send buffer function */

/* #elif STDIN_LINE == 99 /\* USB *\/ */
/*     #warning TODO implement USB send buffer function */
/* #endif */
    
/*     break; */
/*   default: */
/*     /\* return an error for all other FD's *\/ */
/*     errno = EBADF; */
/*     return -1; */
/*   } */
  return 0;
}

/*
 *  Status of a file (by name).
 */
int _stat(const char *filepath, struct stat *st) {
    st->st_mode = S_IFCHR;
    return 0;
}

/*
 * Timing information for current process. Minimal implementation:
 */
clock_t _times(struct tms *buf) {
    return -1;
}

/*
 *  Remove a file's directory entry. Minimal implementation:
 */
int _unlink(char *name) {
    errno = ENOENT;
    return -1;
}

/*
 * Wait for a child process. Minimal implementation:
 */
int _wait(int *status) {
    errno = ECHILD;
    return -1;
}

/*
 * Write a character to a file. `libc' subroutines will use this
 * system routine for output to all files, including stdout Returns -1
 * on error or number of bytes sent
 */
int _write(int file, char *ptr, int len) {

#ifdef USE_USB
  uint8_t* u8ptr = (uint8_t*)ptr;
#endif /* USE_USB */

  int i;
  switch (file) {
    /*
     * STDOUT
     */
  case STDOUT_FILENO: 

#if STDOUT_LINE == 3 /* USART3 */
    for (i = 0; i < len; i++) {
      usart3_writeb((unsigned char)ptr[i]);
    }
#warning TODO implement usart3-send-buffer Function

#elif STDOUT_LINE == 4 /* UART4 */
    for (i = 0; i < len; i++) {
      uart4_writeb((unsigned char)ptr[i]);
    }
#warning TODO implement uart4 send buffer function


#elif STDOUT_LINE == 99 /* USB */
    APP_FOPS.pIf_DataTx(u8ptr, len);
#endif

        break;

	/*
	 * STDERR
	 */
	case STDERR_FILENO: /* stderr */

#if STDERR_LINE == 3 /* USART3 */
    for (i = 0; i < len; i++) {
      usart3_writeb((unsigned char)ptr[i]);
    }
#warning TODO implement usart3-send-buffer Function

#elif STDERR_LINE == 4 /* UART4 */
    for (i = 0; i < len; i++) {
      uart4_writeb((unsigned char)ptr[i]);
    }
#warning TODO implement uart4 send buffer function


#elif STDERR_LINE == 99 /* USB */
    APP_FOPS.pIf_DataTx(u8ptr, len);
#warning USB wirte not implemente
#endif


        break;
    default:
        errno = EBADF;
        return -1;
    }
    return 0;
}
