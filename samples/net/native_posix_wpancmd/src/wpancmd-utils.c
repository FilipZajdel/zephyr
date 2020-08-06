#include <zephyr.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include "wpancmd-utils.h"

struct request_command commands[] = { { RESET, "reset" },
				      { TX, "tx" },
				      { SET_CHANNEL, "setchannel" },
				      { START, "start" },
				      { STOP, "stop" },
				      { SET_SHORT_ADDR, "setshortaddr" },
				      { SET_PAN_ID, "setpanid" },
				      { SET_IEEE_ADDR, "setieeeaddr" },
				      { SET_TXPOWER, "settxpower" },
				      { SHUT_DOWN, "quit" },
				      { SHUT_DOWN, "exit" },
				      { SHUT_DOWN, "q" },
				      { SHUT_DOWN, "out" },
				      { HELP, "help" },
				      { HELP, "h" } };

#define COMMANDS_NUM ARRAY_SIZE(commands);

/**
 * Attempts to get a line from stdin until @timeout_s time passes.
 * 
 * Arguments:
 * dest [out]	-	The data taken from stdin will be stored here. 
 * 					If dest is NULL, then the data will be allocated and must
 * 					be deallocated with a free(*dest) invocation.
 * size			-	The size of provided buffer.
 * timeout_s	-	The time to wait until function returns.
 * 
 * Returns:
 * negative		-	The errno code - ETIME on timeout, ENOMEM when either
 * 					memory allocation fails or provided buffer is too small.
 * 0 or positive-	The number of bytes read.
 */
int getline_timeout(char **dest, unsigned int *size, unsigned long timeout_s)
{
	struct timeval tv = { .tv_sec = timeout_s, .tv_usec = 0 };
	fd_set set;
	int ret;

	FD_ZERO(&set);
	FD_SET(STDIN_FILENO, &set);

	ret = select(1, &set, NULL, NULL, &tv);

	if (ret > 0) {
		char buf[128] = "";

		int nread = read(STDIN_FILENO, buf, 127);
		buf[nread] = 0;
		if (!(*dest)) {
			*dest = malloc(nread * sizeof(**dest));

			if (!*dest) {
				return -ENOMEM;
			}
		} else if (*size < nread) {
			return -ENOMEM;
		}

		strncpy(*dest, buf, nread);
		ret = nread;
	} else {
		ret = -ETIME;
	}

	return ret;
}

/**
 * Checks if two strings are identical from the beginning to
 * delimiter in str1.
 * 
 * Arguments:
 * str1         -       Str to compare
 * str2         -       Str to compare
 * delimiter    -       The character terminating comparison
 * 
 * Returns:
 * 0            -       str1 and str2 are identical until delimiter
 * negative     -       Provided strings don't match
 */
int strcmp_until(const char *str1, const char *str2, const char delimiter)
{
	int i = 0;
	int delim_at = 0;

	do {
		delim_at++;
	} while ((str1[delim_at] != delimiter) && delim_at < strlen(str1));

	for (i = 0; i < delim_at; i++) {
		if (str1[i] != str2[i]) {
			return -1;
		}
	}

	return 0;
}

/**
 * Search the string for the specified character
 * 
 * Return index in string or -1 if there is no such character
 */
int find_first_in_str(const char *str, char ch_to_match)
{
	for (int i = 0; i < strlen(str); i++) {
		if (str[i] == ch_to_match) {
			return i;
		}
	}

	return -1;
}

/** Replace old_char with new_char in a giver str_to_replace */
int str_replace(char *str_to_replace, char old_char, char new_char)
{
	int repl_ctr;
	char *p_str;

	for (repl_ctr = 0, p_str = str_to_replace; strlen(p_str); p_str++) {
		if (*p_str == old_char) {
			*p_str = new_char;
			repl_ctr++;
		}
	}

	return repl_ctr;
}

/** Cuts out a character from str and replaces it with empty (concatenating) */
void str_cut(char *str_to_cut, char char_to_cut)
{
	for (char *p_str = str_to_cut; strlen(p_str); p_str++) {
		if (*p_str == char_to_cut) {
			char buf[strlen(p_str)];
			strcpy(buf, p_str + 1);
			strcpy(p_str, buf);
		}
	}
}

/** Decodes the command in given string
 * 
 * Arguments:
 * cmd                  -       String to decode
 * arg_idx [out]        -       If command has arguments, their starting index
 *                              will be returned via this variable.
 * args_len             -       If command has arguments, their length will
 *                              be returned via this variable.
 * 
 * Returns:
 * one of wpanusb_requests     - On success
 * negative                    - On recognising command failure
 */
enum wpanusb_requests cmd_decode(const char *cmd, uint8_t *arg_idx,
				 uint8_t *args_len)
{
	enum wpanusb_requests request = -1;

        str_cut(cmd, '\n');
	for (int i = 0; i < sizeof(commands) / sizeof(*commands); i++) {
		if (strcmp_until(cmd, commands[i].command,
				 COMMAND_ARG_DELIIMITER) == 0) {
			request = commands[i].request;

			*arg_idx =
				find_first_in_str(cmd, COMMAND_ARG_DELIIMITER) +
				1;
			*args_len = strlen(cmd + *arg_idx);
			break;
		}
	}

	return request;
}

/** Prints all commands to the stdout */
void print_commands(void)
{
	for (int i = 0; i < ARRAY_SIZE(commands); i++) {
		printk("%s\n", commands[i].command);
	}
}