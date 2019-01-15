all: RDB_reader

RDB_reader:
	gcc -I./ -o RDB_reader RDB_reader.c

host:
	gcc -I./ -o RDB_reader_host RDB_reader.c -DUSEHOST=1

csum:
	gcc -o csum csum.c

clean:
	rm RDB_reader RDB_reader_host csum
