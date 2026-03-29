CC = clang
CFLAGS = -std=c23 -Wall -Wextra -g

%_test: %_test.o utest.h
	$(CC) $< -o $@
	@./$@
	@rm $@

%.o: %.c utest.h
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	@rm -f *.o
