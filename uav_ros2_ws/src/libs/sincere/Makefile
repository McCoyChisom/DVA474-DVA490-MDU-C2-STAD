CC=cc 
LIBS=-lrt
CFLAGS=-std=c99 -Wall -Wextra -pedantic -D_XOPEN_SOURCE=500 -fPIC

# Define the name of the shared library
SHARED_LIB=lib_sincere.so

# Targets for the shared library and tests
all: ${SHARED_LIB} ${TESTS}

# Rule to create the shared library
${SHARED_LIB}: sincere.c
	${CC} -shared -o ${SHARED_LIB} sincere.c ${LIBS}

# Specify how to build the tests
TESTS=write read

# Build the write and read executables
write: write.c ${SHARED_LIB}
	${CC} ${CFLAGS} write.c -o write -L. -l_sincere ${LIBS}

read: read.c ${SHARED_LIB}
	${CC} ${CFLAGS} read.c -o read -L. -l_sincere ${LIBS}

clean:
	rm -f ${TESTS} ${SHARED_LIB}

