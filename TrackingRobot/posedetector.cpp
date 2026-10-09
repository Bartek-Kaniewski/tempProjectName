#include "posedetector.h"
#include <cmath>
#include <algorithm>

const std::vector<std::pair<int, int>> SKELETON_PAIRS = {
    {3, 4}, {0, 3}, {0, 4},
    {5, 7}, {7, 9},         // Lewe ramię
    {6, 8}, {8, 10},        // Prawe ramię
    {5, 6}, {5, 11}, {6, 12}, {11, 12} // Tułów
};

const std::vector<int> ESSENTIAL_JOINTS = {0, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};

PoseDetector::PoseDetector() {}

bool PoseDetector::loadModel(const std::string &modelPath) {
    try {
        m_net = cv::dnn::readNetFromONNX(modelPath);
        m_net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        m_net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        return !m_net.empty();
    } catch (const cv::Exception &) {
        return false;
    }
}

float PoseDetector::calculateAngle(const KeyPoint &a, const KeyPoint &b, const KeyPoint &c) {
    float v1x = a.x - b.x;
    float v1y = a.y - b.y;
    float v2x = c.x - b.x;
    float v2y = c.y - b.y;

    float dot = v1x * v2x + v1y * v2y;
    float mag1 = std::hypot(v1x, v1y);
    float mag2 = std::hypot(v2x, v2y);

    if (mag1 * mag2 < 1e-4) return 0.0f;

    float cosAngle = dot / (mag1 * mag2);
    cosAngle = std::clamp(cosAngle, -1.0f, 1.0f);
    return std::acos(cosAngle) * 180.0f / 3.14159265f;
}

std::vector<PersonPose> PoseDetector::processFrame(cv::Mat &frame) {
    std::vector<PersonPose> poses;
    if (m_net.empty() || frame.empty()) return poses;

    cv::Mat blob;
    cv::dnn::blobFromImage(frame, blob, 1.0 / 255.0, cv::Size(m_inputWidth, m_inputHeight), cv::Scalar(), true, false);
    m_net.setInput(blob);

    std::vector<cv::Mat> outputs;
    m_net.forward(outputs, m_net.getUnconnectedOutLayersNames());

    if (outputs.empty()) return poses;

    cv::Mat output = outputs[0];
    int dimensions = output.size[1];
    int rows = output.size[2];

    output = output.reshape(1, dimensions);
    cv::transpose(output, output);

    float x_factor = (float)frame.cols / m_inputWidth;
    float y_factor = (float)frame.rows / m_inputHeight;

    float maxScore = 0.0f;
    int bestRow = -1;

    for (int i = 0; i < rows; ++i) {
        float score = output.at<float>(i, 4);
        if (score > m_confThreshold && score > maxScore) {
            maxScore = score;
            bestRow = i;
        }
    }

    if (bestRow != -1) {
        PersonPose pose;
        pose.boxScore = maxScore;

        for (int k = 0; k < 17; ++k) {
            int kptIdx = 5 + k * 3;
            float kptX = output.at<float>(bestRow, kptIdx) * x_factor;
            float kptY = output.at<float>(bestRow, kptIdx + 1) * y_factor;
            float kptConf = output.at<float>(bestRow, kptIdx + 2);

            pose.keypoints.push_back({kptX, kptY, kptConf});
        }

        // =========================================================================
        // 1. UKŁAD ODNIESIENIA CIAŁA CZŁOWIEKA (ODPORNY NA POCHYLENIE KAMERY)
        // =========================================================================
        if (pose.keypoints[5].confidence > m_kptThreshold &&
            pose.keypoints[6].confidence > m_kptThreshold) {

            // Baza klatki piersiowej (Bark Lewy 5 -> Bark Prawy 6)
            float chestDx = pose.keypoints[6].x - pose.keypoints[5].x;
            float chestDy = pose.keypoints[6].y - pose.keypoints[5].y;
            float chestWidth = std::hypot(chestDx, chestDy);
            if (chestWidth < 20.0f) chestWidth = 100.0f;

            // Wektory kierunkowe tułowia:
            float bodyDownX = -chestDy / chestWidth;
            float bodyDownY =  chestDx / chestWidth;
            float bodyRightX = chestDx / chestWidth;
            float bodyRightY = chestDy / chestWidth;

            // --- A. KĄT UNIESIENIA BARKU (GÓRA / DÓŁ) ---
            if (pose.keypoints[8].confidence > m_kptThreshold) {
                float armDx = pose.keypoints[8].x - pose.keypoints[6].x;
                float armDy = pose.keypoints[8].y - pose.keypoints[6].y;

                float projDown  = armDx * bodyDownX + armDy * bodyDownY;
                float projRight = armDx * bodyRightX + armDy * bodyRightY;

                float elevationRad = std::atan2(std::abs(projRight), projDown);
                if (projDown < 0) {
                    elevationRad = 3.14159265f - std::atan2(std::abs(projRight), -projDown);
                }
                pose.rightShoulderPitch = std::clamp(elevationRad * 180.0f / 3.14159265f, 0.0f, 180.0f);
            }

            // --- B. KĄT WYCIĄGNIĘCIA W PRZÓD (GŁĘBIA) ---
            if (pose.keypoints[10].confidence > m_kptThreshold) {
                float armSpan2D = std::hypot(pose.keypoints[10].x - pose.keypoints[6].x,
                                             pose.keypoints[10].y - pose.keypoints[6].y);
                float expectedSpan = chestWidth * 1.6f;
                float ratio = std::clamp(armSpan2D / expectedSpan, 0.0f, 1.0f);
                pose.rightShoulderYaw = (1.0f - ratio) * 80.0f; // 0° z boku -> 80° w przód
            }

            // --- C. ZGIĘCIE ŁOKCIA (Staw 4: 0° wyprost, 90° zgięty) ---
            if (pose.keypoints[8].confidence > m_kptThreshold &&
                pose.keypoints[10].confidence > m_kptThreshold) {

                float rawAngle = calculateAngle(pose.keypoints[6], pose.keypoints[8], pose.keypoints[10]);
                float flexionDeg = 180.0f - rawAngle; // 180 - 180 = 0° (prosta!), 180 - 90 = 90° (zgięta)
                pose.rightElbowAngle = std::clamp(flexionDeg, 0.0f, 135.0f);
            }

            // --- D. ROTACJA RAMIENIA (BICEPS W PIONIE) ---
            if (pose.keypoints[8].confidence > m_kptThreshold &&
                pose.keypoints[10].confidence > m_kptThreshold) {

                float forearmDy = pose.keypoints[10].y - pose.keypoints[8].y;
                if (forearmDy < -20.0f) {
                    pose.torsoRoll = 90.0f; // Dłoń w górze -> biceps pionowo!
                } else {
                    pose.torsoRoll = 0.0f;
                }
            }
        }

        drawSkeleton(frame, pose);
        poses.push_back(pose);
    }

    return poses;
}

void PoseDetector::drawSkeleton(cv::Mat &frame, const PersonPose &pose) {
    for (const auto &pair : SKELETON_PAIRS) {
        const auto &p1 = pose.keypoints[pair.first];
        const auto &p2 = pose.keypoints[pair.second];

        if (p1.confidence > m_kptThreshold && p2.confidence > m_kptThreshold) {
            cv::Scalar color = (pair.first <= 4 && pair.second <= 4) ? cv::Scalar(255, 200, 0) : cv::Scalar(0, 255, 0);
            cv::line(frame, cv::Point(p1.x, p1.y), cv::Point(p2.x, p2.y), color, 2, cv::LINE_AA);
        }
    }

    for (int idx : ESSENTIAL_JOINTS) {
        const auto &kpt = pose.keypoints[idx];
        if (kpt.confidence > m_kptThreshold) {
            cv::circle(frame, cv::Point(kpt.x, kpt.y), 5, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
            cv::circle(frame, cv::Point(kpt.x, kpt.y), 7, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        }
    }

    if (pose.rightElbowAngle >= 0) {
        const auto &elbow = pose.keypoints[8];
        std::string txt = "E: " + std::to_string((int)pose.rightElbowAngle) + " deg";
        cv::putText(frame, txt, cv::Point(elbow.x + 10, elbow.y - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    }

    if (pose.rightShoulderPitch >= 0) {
        const auto &shoulder = pose.keypoints[6];
        std::string txt = "S: " + std::to_string((int)pose.rightShoulderPitch) + " deg";
        cv::putText(frame, txt, cv::Point(shoulder.x + 10, shoulder.y - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 200, 255), 2, cv::LINE_AA);
    }
}