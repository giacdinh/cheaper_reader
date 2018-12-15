all:

RDB_reader:
	gcc -I./ -o RDB_reader RDB_reader.c

csum:
	gcc -o csum csum.c

clean:
	rm RDB_reader csum
