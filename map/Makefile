.PHONY: test clean

test:
	g++ -std=c++11 -c -w test.cpp 
	g++ test.o -o test
	./test

clean:
	rm test || True
	rm test.o || True