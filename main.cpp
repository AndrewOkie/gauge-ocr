//
//  main.cpp
//  gauge-ocr
//
//  Created by Andrey Perevozchikov on 17.04.17.
//

#include "platform.h"
#include "detector.h"

int main(int argc, const char *argv[])
{
    cv::CommandLineParser parser(argc, argv,
        "{ help h usage ? |                                  | show this help message }"
        "{ @in            | ./data/sample.mp4                | }"
        "{ @out           | appsrc ! videoconvert ! fakesink | }"
    );

    if (parser.has("help"))
    {
        parser.printMessage();
        return 0;
    }

    cv::VideoCapture capture(parser.get< std::string >("@in"));
    if (!capture.isOpened())
    {
        parser.printMessage();
        std::cerr << "ERR: Can't open the input video stream!" << std::endl;
        return -1;
    }

    cv::Mat frame;
    capture >> frame;
    if (frame.empty())
    {
        std::cerr << "ERR: Blank frame grabbed!" << std::endl;
        return -1;
    }
#if 0
    bool isColor = (frame.type() == CV_8UC3);
    cv::VideoWriter writer(parser.get< std::string >("@out"), 0, 30.0, frame.size(), isColor);
    if (!writer.isOpened())
    {
        parser.printMessage();
        std::cerr << "ERR: Can't open the output stream for write!" << std::endl;
        return -1;
    }
#endif
    MarkerDetector markerDetector;
    std::vector< Marker > markers;

    int key = 0;
    while (key != 'q')
    {
        double tick_start = (double)cv::getTickCount();

        capture >> frame;
        if (frame.empty())
        {
            break;
        }

        cv::resize(frame, frame, cv::Size(), 0.5, 0.5);

        markerDetector.ProcessFrame(frame, markers);

        for (int i = 0; i < markers.size(); i++)
        {
            Marker marker = markers[i];
            std::vector< cv::Point2f > pos = marker.GetPos();

            cv::line(frame, pos[0], pos[1], cv::Scalar(0, 255, 0), 1, CV_AA);
            cv::line(frame, pos[1], pos[2], cv::Scalar(0, 255, 0), 1, CV_AA);
            cv::line(frame, pos[2], pos[3], cv::Scalar(0, 255, 0), 1, CV_AA);
            cv::line(frame, pos[3], pos[0], cv::Scalar(0, 255, 0), 1, CV_AA);

            cv::circle(frame, pos[0], 3, cv::Scalar(255, 0, 0), 1, CV_AA);
            cv::circle(frame, pos[1], 3, cv::Scalar(0, 255, 0), 1, CV_AA);
            cv::circle(frame, pos[2], 3, cv::Scalar(0, 0, 255), 1, CV_AA);
            cv::circle(frame, pos[3], 3, cv::Scalar(255, 255, 255), 1, CV_AA);

            char szMarkerId[23];
            sprintf(szMarkerId, "id: %d", marker.GetId());
            cv::putText(frame, szMarkerId, pos[0] - cv::Point2f(0, 10), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);

            char szMarkerValue[20];
            sprintf(szMarkerValue, "value: %.4f", marker.GetValue());
            cv::putText(frame, szMarkerValue, pos[0] - cv::Point2f(-80, 10), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
        }

        double fps = cv::getTickFrequency() / ((double)cv::getTickCount() - tick_start);

        char szFPS[13];
        sprintf(szFPS, "FPS: %4.2f", fps);
        cv::putText(frame, szFPS, cv::Point(20, 30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
#if 0
        writer << frame;
#endif
        cv::imshow("frame", frame);

        key = cv::waitKey(1);
    }

    return 0;
}
