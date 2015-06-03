
int aes(int fd);
int des(int fd);
int expo(int fd);
int pk(int fd);
int hash(int fd);
int arc4(int fd);
int kasumi(int fd);

void dumpm(unsigned char *csData, unsigned int nBytes);

#define consolemsg(modnm, submodnm, cyclenm, state) \
printf("%-8s %-16s %-32s -> %-16s\n", modnm, submodnm, cyclenm, state)
