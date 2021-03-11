#include <iostream>     // cout, cin, streambuf, hex, endl, sgetc, sbumpc
#include <iomanip>      // setw, setfill
#include <fstream>      // fstream

// These inclusions required to set terminal mode.
#include <termios.h>    // struct termios, tcgetattr(), tcsetattr()
#include <stdio.h>      // perror(), stderr, stdin, fileno()
#include <thread>

#include <ros/ros.h>

class UserDriver {
    public:
    UserDriver(ros::NodeHandle nh) : thread() {
        move_pub = nh.advertise<std_msgs::String>("control_move", 50);

        qr_sub = nh.subscribe("camera_pkg_node/qr_code", 50, &qrCallback);
    }

    bool input_loop(ros::NodeHandle nh) {
        struct termios t;
        struct termios t_saved;

        // Set terminal to single character mode.
        tcgetattr(fileno(stdin), &t);
        t_saved = t;
        t.c_lflag &= (~ICANON & ~ECHO);
        t.c_cc[VTIME] = 0;
        t.c_cc[VMIN] = 1;
        if (tcsetattr(fileno(stdin), TCSANOW, &t) < 0) {
            perror("Unable to set terminal to single character mode");
            return -1;
        }
        std::streambuf *pbuf = cin.rdbuf();
        bool done = false;
        while (nh.ok()) {
            // Read single characters from cin.
            cout << "Enter an character (or esc to quit): " << endl;
            char c;
            if (pbuf->sgetc() == EOF) done = true;
            c = pbuf->sbumpc();
            if (c == 0x1b) {
                done = true;
            } else {
                cout << "You entered character 0x" << setw(2) << setfill('0') << hex << int(c) << "'" << endl;
            }

            std_msgs::String msg;
            msg.data = &c;
            move_pub(msg);
        }
        // Restore terminal mode.
        if (tcsetattr(fileno(stdin), TCSANOW, &t_saved) < 0) {
            perror("Unable to restore terminal mode");
            return -1;
        }

        return 0;

    }

    static void qrCallback(std_msgs::String msg) {
        std::cout << msg.data << std::endl;
    }

    static void rosLoop() {
        ros::Rate loop_rate(50);
        while (ros::ok())
        {
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    private:
    ros::Publisher<std_msgs::String> move_pub;
    ros::Subscriber<std_msgs::String> qr_sub
};