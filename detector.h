//
//  detector.h
//  gauge-ocr
//
//  Created by Andrey Perevozchikov on 17.04.17.
//

#ifndef DETECTOR_H
#define DETECTOR_H

#include "platform.h"
#include "marker.h"

class MarkerDetector
{
public:
    MarkerDetector();

    void ProcessFrame(cv::Mat frame, std::vector< Marker > &markers);

protected:
    void FindMarkers(std::vector< Marker > &markers);
    void FindCandidates(std::vector< std::vector< cv::Point2f > > &candidates);
    void RecognizeCandidates(std::vector< std::vector< cv::Point2f > > candidates, std::vector< Marker > &markers);

private:
    cv::Mat m_frame;
    cv::Mat m_gray;
    cv::Mat m_threshold;
};

#endif // DETECTOR_H
