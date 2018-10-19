//
//  marker.h
//  gauge-ocr
//
//  Created by Andrey Perevozchikov on 17.04.17.
//

#ifndef MARKER_H
#define MARKER_H

#include "platform.h"

class Marker
{
public:
    Marker();

    int GetId();
    void SetId(int id);

    float GetValue();
    void SetValue(float value);

    std::vector< cv::Point2f > GetPos();
    void SetPos(std::vector< cv::Point2f > pos);

private:
    int m_id;
    float m_value;
    std::vector< cv::Point2f > m_pos;
};

#endif // MARKER_H
