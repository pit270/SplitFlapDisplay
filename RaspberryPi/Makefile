
default: splitflap

splitflap:
	g++ main.cpp pca9685/src/pca9685.c -lwiringPi -pthread -o SplitFlap

servotest:
	g++ servotest.cpp pca9685/src/pca9685.c -lwiringPi -pthread -o ServoTest

clean:
	-rm -f SplitFlap
	-rm -f ServoTest