//
//  marker.cpp
//  gauge-ocr
//
//  Created by Andrey Perevozchikov on 17.04.17.
//

#include "platform.h"
#include "marker.h"

Marker::Marker() : m_id( -1 )
{
}

int Marker::GetId()
{
    return m_id;
}

void Marker::SetId( int id )
{
    m_id = id;
}

float Marker::GetValue()
{
    return m_value;
}

void Marker::SetValue(float value)
{
    m_value = value;
}

std::vector< cv::Point2f > Marker::GetPos()
{
    return m_pos;
}

void Marker::SetPos(std::vector< cv::Point2f > pos)
{
    m_pos = pos;
}
