//
//  detector.cpp
//  gauge-ocr
//
//  Created by Andrey Perevozchikov on 17.04.17.
//

#include "platform.h"
#include "detector.h"

//------------------------------------------------------------------------------
// Configuration
//------------------------------------------------------------------------------
const double minPerimeterRate = 0.03;
const double maxPerimeterRate = 4.0;
const double accuracyRate = 0.03;
const double minCornerDistanceRate = 0.05;
const int minDistanceToBorder = 3;
const int cornerRefinementWinSize = 5;
const int cornerRefinementMaxIterations = 30;
const double cornerRefinementMinAccuracy = 0.1;
const int markerBorderBits = 1;
const int markerPosBits = 2;
const int markerInfoBits = 8;
const int perspectiveRemovePixelPerCell = 12;
const double perspectiveRemoveCellMarginRate = 0.13;

const int roiImageSize = 600;
const int gaugeImageSize = roiImageSize / 2;
const int centerPointRadius = roiImageSize / 15;
const int minLineThreshold = 50;
const double minLineLength =  roiImageSize * 0.15;

//------------------------------------------------------------------------------
// Utils
//------------------------------------------------------------------------------
int calcHistMedian(cv::Mat channel)
{
    int histSize = 256;
    float range[] = { 0, 256 };
    const float *histRange = { range };

    cv::Mat hist;
    cv::calcHist(&channel, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, true, false);

    int mid = (channel.rows * channel.cols) / 2;
    int bin = 0;
    int median = -1;
    for (int i = 0; i < histSize && median < 0; i++)
    {
        bin += cvRound(hist.at< float >(i));
        if (bin > mid)
            median = i;
    }

    return median;
}

void autoCanny(cv::Mat in, cv::Mat &out, double sigma = 0.33)
{
    int median = calcHistMedian(in);

    int lower = (MAX(0.0, (1.0 - sigma) * median));
    int upper = (MIN(255.0, (1.0 + sigma) * median));

    cv::Canny(in, out, lower, upper);
}

bool isPointInCircle(cv::Point pt, cv::Point center, int radius)
{
    if (pt.inside(cv::Rect(center.x - radius, center.y - radius, 2 * radius, 2 * radius)))
    {
        int dx = pow(center.x - pt.x, 2);
        int dy = pow(center.y - pt.y, 2);

        return (dx + dy) <= pow(radius, 2);
    }

    return false;
}

bool lineSlope(cv::Point pt1, cv::Point pt2, float &slope)
{
    float dx = pt2.x - pt1.x;
    float dy = pt2.y - pt1.y;

    // make sure we are not dividing by zero
    if (dy != 0)
    {
        slope = dy / dx;
        return true;
    }
    else
    {
        slope = 0.0f;
        return false;
    }
}

MarkerDetector::MarkerDetector()
{
}

void MarkerDetector::ProcessFrame(cv::Mat frame, std::vector< Marker > &markers)
{
    frame.copyTo( m_frame );
    markers.clear();

    FindMarkers( markers );
}

void MarkerDetector::FindMarkers(std::vector< Marker > &markers)
{
    cv::cvtColor(m_frame, m_gray, cv::COLOR_BGR2GRAY);

    cv::threshold(m_gray, m_threshold, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
//    adaptiveThreshold(m_gray, m_threshold, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 49, 0);

    std::vector< std::vector< cv::Point2f > > candidates;
    FindCandidates(candidates);

#ifdef DEBUG
    std::cout << "candidates.size(): " << candidates.size() << std::endl;
#endif

    RecognizeCandidates(candidates, markers);
}

void MarkerDetector::FindCandidates(std::vector< std::vector< cv::Point2f > > &candidates)
{
    std::vector< std::vector< cv::Point > > contours;
    std::vector< cv::Vec4i > hierarchy;
    cv::findContours(m_threshold, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

    for (size_t i = 0; i < contours.size(); i++)
    {
        // check hierarchy
        int c = 0;

        int k = i;
        while (hierarchy[k][2] != -1)
        {
            k = hierarchy[k][2] ;
            c++;
        }

        if (hierarchy[k][2] != -1)
        {
            c++;
        }

        if (c < 3)
            continue;

        // check perimeter
        size_t minPerimeterPixels = (size_t)(minPerimeterRate * MAX(m_threshold.cols, m_threshold.rows));
        size_t maxPerimeterPixels = (size_t)(maxPerimeterRate * MAX(m_threshold.cols, m_threshold.rows));
        if (contours[i].size() < minPerimeterPixels || contours[i].size() > maxPerimeterPixels)
            continue;

        // check if countor is square and convex
        std::vector< cv::Point > approxCurve;
        cv::approxPolyDP(contours[i], approxCurve, double(contours[i].size()) * accuracyRate, true);
        if (approxCurve.size() != 4 || !cv::isContourConvex(approxCurve))
            continue;

        // check min distance between corners
        double minDistSq = MAX(m_threshold.cols, m_threshold.rows) * MAX(m_threshold.cols, m_threshold.rows);
        for (int j = 0; j < 4; j++)
        {
            double d = (double)(approxCurve[j].x - approxCurve[(j + 1) % 4].x) *
                       (double)(approxCurve[j].x - approxCurve[(j + 1) % 4].x) +
                       (double)(approxCurve[j].y - approxCurve[(j + 1) % 4].y) *
                       (double)(approxCurve[j].y - approxCurve[(j + 1) % 4].y);

            minDistSq = MIN(minDistSq, d);
        }
        double minCornerDistancePixels = double(contours[i].size()) * minCornerDistanceRate;
        if (minDistSq < minCornerDistancePixels * minCornerDistancePixels)
            continue;

        // check if it is too near to the image border
        bool tooNearBorder = false;
        for (int j = 0; j < 4; j++)
        {
            if (approxCurve[j].x < minDistanceToBorder || approxCurve[j].y < minDistanceToBorder ||
                approxCurve[j].x > m_threshold.cols - 1 - minDistanceToBorder ||
                approxCurve[j].y > m_threshold.rows - 1 - minDistanceToBorder)
                tooNearBorder = true;
        }
        if (tooNearBorder)
            continue;

        // push candidate
        std::vector< cv::Point2f > currentCandidate;
        currentCandidate.resize(4);
        for (int j = 0; j < 4; j++)
        {
            currentCandidate[j] = cv::Point2f((float)approxCurve[j].x, (float)approxCurve[j].y);
        }

        // assure order of candidate corners is clockwise
        float dx1 = currentCandidate[1].x - currentCandidate[0].x;
        float dy1 = currentCandidate[1].y - currentCandidate[0].y;
        float dx2 = currentCandidate[2].x - currentCandidate[0].x;
        float dy2 = currentCandidate[2].y - currentCandidate[0].y;
        float crossProduct = (dx1 * dy2) - (dy1 * dx2);

        if (crossProduct < 0.0f)
        {
            swap(currentCandidate[1], currentCandidate[3]);
        }

        candidates.push_back(currentCandidate);
    }
}

void MarkerDetector::RecognizeCandidates(std::vector< std::vector< cv::Point2f > > candidates, std::vector< Marker > &markers)
{
    for (size_t i = 0; i < candidates.size(); i++)
    {
        cv::cornerSubPix(m_gray, candidates[i], cv::Size(cornerRefinementWinSize, cornerRefinementWinSize), cv::Size(-1, -1),  cv::TermCriteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, cornerRefinementMaxIterations, cornerRefinementMinAccuracy));

        // number of bits in the marker
        int markerSizeWithBorders = markerInfoBits + 2 * (markerPosBits + markerBorderBits);
        int cellMarginPixels = int(perspectiveRemoveCellMarginRate * perspectiveRemovePixelPerCell);

        // marker image after removing perspective
        cv::Mat resultImg;
        int resultImgSize = markerSizeWithBorders * perspectiveRemovePixelPerCell;

        cv::Mat resultImgCorners(4, 1, CV_32FC2);
        resultImgCorners.ptr< cv::Point2f >(0)[0] = cv::Point2f(0, 0);
        resultImgCorners.ptr< cv::Point2f >(0)[1] = cv::Point2f((float)resultImgSize - 1, 0);
        resultImgCorners.ptr< cv::Point2f >(0)[2] = cv::Point2f((float)resultImgSize - 1, (float)resultImgSize - 1);
        resultImgCorners.ptr< cv::Point2f >(0)[3] = cv::Point2f(0, (float)resultImgSize - 1);

        // remove perspective
        cv::Mat transformation = cv::getPerspectiveTransform(candidates[i], resultImgCorners);
        cv::warpPerspective(m_gray, resultImg, transformation, cv::Size(resultImgSize, resultImgSize));

        // now extract code, first threshold using Otsu
        cv::threshold(resultImg, resultImg, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);

        // for each cell
        cv::Mat bits(markerSizeWithBorders, markerSizeWithBorders, CV_8UC1, cv::Scalar::all(0));
        for (int y = 0; y < markerSizeWithBorders; y++)
        {
            for (int x = 0; x < markerSizeWithBorders; x++)
            {
                int Xstart = x * (perspectiveRemovePixelPerCell) + cellMarginPixels;
                int Ystart = y * (perspectiveRemovePixelPerCell) + cellMarginPixels;
                cv::Mat square = resultImg(cv::Rect(Xstart, Ystart, perspectiveRemovePixelPerCell - 2 * cellMarginPixels, perspectiveRemovePixelPerCell - 2 * cellMarginPixels));
                // count white pixels on each cell to assign its value
                size_t nZ = (size_t)cv::countNonZero(square);
                if (nZ > square.total() / 2)
                {
                    bits.at< unsigned char >(y, x) = 1;
                }
                else
                {
                    bits.at< unsigned char >(y, x) = 0;
                }
            }
        }

        // check if borders are valid
        bool validBorders = true;
        for (int j = 0; j < markerSizeWithBorders; j++)
        {
            if (!bits.at< unsigned char >(0, i))
                validBorders = false;

            if (!bits.at< unsigned char >(i, markerSizeWithBorders - 1))
                validBorders = false;

            if (!bits.at< unsigned char >(markerSizeWithBorders - 1, i))
                validBorders = false;

            if (!bits.at< unsigned char >(i, 0))
                validBorders = false;
        }
        if (!validBorders)
        {
            continue;
        }

        // update corners orientation
        int tlCornerNum = 2 * bits.at< unsigned char >(markerBorderBits, markerBorderBits) + bits.at< unsigned char >(markerBorderBits, markerBorderBits + 1);
        if (tlCornerNum == 1) // west
        {
            std::rotate(candidates[i].begin(), candidates[i].begin() + 3, candidates[i].end());
            cv::transpose(bits, bits);
            cv::flip(bits, bits, 1);
        }
        else if (tlCornerNum == 2) // south
        {
            std::rotate(candidates[i].begin(), candidates[i].begin() + 2, candidates[i].end());
            cv::flip(bits, bits, -1);
        }
        else if (tlCornerNum == 3) // east
        {
            std::rotate(candidates[i].begin(), candidates[i].begin() + 1, candidates[i].end());
            cv::transpose(bits, bits);
            cv::flip(bits, bits, 0);
        }
#ifdef DEBUG
        for (int y = 0; y < bits.rows; y++)
        {
            std::cout << "y: " << y << "\t|";
            for (int x = 0; x < bits.cols; x++)
            {
                std::cout << " " << (bits.at< unsigned char >(y, x) > 0 ? 1 : 0);
            }
            std::cout << std::endl;
        }
#endif
        int cornerNum[4] = {2 * bits.at< unsigned char >(markerBorderBits, markerBorderBits) + bits.at< unsigned char >(markerBorderBits, markerBorderBits + 1),
                            2 * bits.at< unsigned char >(markerBorderBits, markerSizeWithBorders - markerBorderBits - 1) + bits.at< unsigned char >(markerBorderBits + 1, markerSizeWithBorders - markerBorderBits - 1),
                            2 * bits.at< unsigned char >(markerSizeWithBorders - markerBorderBits - 1, markerSizeWithBorders - markerBorderBits - 1) + bits.at< unsigned char >(markerSizeWithBorders - markerBorderBits - 1, markerSizeWithBorders - markerBorderBits - 1 - 1),
                            2 * bits.at< unsigned char >(markerSizeWithBorders - markerBorderBits - 1, markerBorderBits) + bits.at< unsigned char >(markerSizeWithBorders - markerBorderBits - 1 - 1, markerBorderBits)};
        if (cornerNum[0] != 0 || cornerNum[1] != 1 || cornerNum[2] != 2 || cornerNum[3] != 3)
        {
            continue;
        }

        unsigned char markerData[4 * markerInfoBits];
        for (int j = 0; j < markerInfoBits; j++)
        {
            markerData[j] = bits.at< unsigned char >(markerBorderBits, markerBorderBits + markerPosBits + j);
            markerData[markerInfoBits + j] = bits.at< unsigned char >(markerBorderBits + markerPosBits + j, markerSizeWithBorders - markerBorderBits - 1);
            markerData[2 * markerInfoBits + j] = bits.at< unsigned char >(markerSizeWithBorders - markerBorderBits - 1, markerBorderBits + markerPosBits + markerInfoBits - 1 - j);
            markerData[3 * markerInfoBits + j] = bits.at< unsigned char >(markerBorderBits + markerPosBits + markerInfoBits - 1 - j, markerBorderBits);
        }

        int markerId = 0;
        for (int j = 0; j < 4 * markerInfoBits; j++)
        {
            markerId = markerId | (markerData[j] << j);
        }
#ifdef DEBUG
        std::cout << "data: ";
        for (int j = 0; j < 4 * markerInfoBits; j++)
        {
            std::cout << (markerData[j] > 0 ? '1' : '0');
        }
        std::cout << endl;

        std::cout << "markerId: " << markerId << std::endl;
#endif
        cv::Mat roiImgCorners(4, 1, CV_32FC2);
        roiImgCorners.ptr< cv::Point2f >(0)[0] = cv::Point2f(0, 0);
        roiImgCorners.ptr< cv::Point2f >(0)[1] = cv::Point2f((float)roiImageSize - 1, 0);
        roiImgCorners.ptr< cv::Point2f >(0)[2] = cv::Point2f((float)roiImageSize - 1, (float)roiImageSize - 1);
        roiImgCorners.ptr< cv::Point2f >(0)[3] = cv::Point2f(0, (float)roiImageSize - 1);

        cv::Mat roiImage;
        cv::Mat roiTransformation = cv::getPerspectiveTransform(candidates[i], roiImgCorners);
        cv::warpPerspective( m_gray, roiImage, roiTransformation, cv::Size(roiImageSize, roiImageSize) );

        cv::Mat gaugeImage = roiImage(cv::Rect((roiImageSize - gaugeImageSize) / 2, (roiImageSize - gaugeImageSize) / 2, gaugeImageSize, gaugeImageSize));

        cv::Mat gaugeEdges;
        autoCanny(gaugeImage, gaugeEdges);

        std::vector< cv::Vec4i > lines;
        cv::HoughLinesP(gaugeEdges, lines, 1, CV_PI / 180.0, minLineThreshold, minLineLength, 2);

        std::vector< cv::Point > center_points;
        std::vector< cv::Point > tip_points;
        for (size_t j = 0; j < lines.size(); j++)
        {
            if (isPointInCircle(cv::Point(lines[j][0], lines[j][1]), cv::Point(gaugeImage.cols / 2, gaugeImage.rows / 2), centerPointRadius))
            {
                center_points.push_back(cv::Point(lines[j][0], lines[j][1]));
                tip_points.push_back(cv::Point(lines[j][2], lines[j][3]));
            }
            else
            {
                center_points.push_back(cv::Point(lines[j][2], lines[j][3]));
                tip_points.push_back(cv::Point(lines[j][0], lines[j][1]));
            }
        }

        if (lines.size() > 0)
        {
            cv::Point first_pt, second_pt;
            for (int j = 0; j < lines.size(); j++)
            {
                first_pt += center_points[j];
                second_pt += tip_points[j];
            }

            first_pt = first_pt / (int)lines.size();
            second_pt = second_pt / (int)lines.size();

            float theta_arrow, arrow_ang;
            if (lineSlope(first_pt, second_pt, theta_arrow))
            {
                arrow_ang = atan(theta_arrow) * 180.0f / CV_PI;
            }
            else
            {
                arrow_ang = -90.0f;
            }

            float theta_null = tan(-45.0f * CV_PI / 180.0f);
            float null_ang = atan(theta_null) * 180.0f / CV_PI;
            float delta_ang = std::abs(arrow_ang - null_ang);

            float pressure = delta_ang * 11.0f / 180.0f;

            // publish
            Marker temp;
            temp.SetId(markerId);
            temp.SetValue(pressure);
            temp.SetPos(candidates[i]);
            markers.push_back(temp);
        }
    }
}
