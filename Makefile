all:
	g++ QR_test.cpp -o libqr.so -shared -fPIC `pkg-config --cflags --libs opencv4`
	gcc pat_follow.c -o pat -L. -lqr -lwiringPi -I.
clean:
	rm libqr.so pat
install:
	sudo cp libqr.so /lib/arm-linux-gnueabihf/
qr:
	g++ QR_test_fast.cpp -o QR_test_fast `pkg-config --cflags --libs opencv4`
threadcompile:
	g++ QR_test_thread.cpp -o libqr.so -shared -fPIC `pkg-config --cflags --libs opencv4`
	gcc pat_follow_thread.c -o pat -L. -lqr -lwiringPi -I. -lpthread
