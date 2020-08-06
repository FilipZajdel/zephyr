#ifndef _WPANCMD_UTILS_H
#define _WPANCMD_UTILS_H

#include <stdint.h>

#define COMMAND_ARG_DELIIMITER ' '
#define COMMAND_LEN (20)

enum wpanusb_requests {
	RESET,
	TX,
	ED,
	SET_CHANNEL,
	START,
	STOP,
	SET_SHORT_ADDR,
	SET_PAN_ID,
	SET_IEEE_ADDR,
	SET_TXPOWER,
	SHUT_DOWN,
    HELP
};

struct request_command {
	enum wpanusb_requests request;
	char command[COMMAND_LEN];
};

int getline_timeout(char **dest, unsigned int *size, unsigned long timeout_s);
int strcmp_until(const char *str1, const char *str2, const char delimiter);
int find_first_in_str(const char *str, char ch_to_match);
int str_replace(char *str_to_replace, char old_char, char new_char);
void str_cut(char *str_to_cut, char char_to_cut);
enum wpanusb_requests cmd_decode(const char *cmd, uint8_t *arg_idx,
				 uint8_t *args_len);
void print_commands(void);


#endif /* _WPANCMD_UTILS_H */
