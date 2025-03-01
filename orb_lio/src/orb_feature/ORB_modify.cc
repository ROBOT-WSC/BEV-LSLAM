#include "orb_feature/ORB_modify.hpp"

ORB_modify::ORB_modify(const ros::NodeHandle &nh)
{
    cout << endl << "Camera Parameters: " << endl;
    // Load ORB parameters
    int nFeatures;
    double fScaleFactor;
    int nLevels;
    int fIniThFAST;
    int fMinThFAST;
    nh.param<int>("orb_lio/nFeatures", nFeatures, 1000);
    nh.param<double>("orb_lio/fScaleFactor", fScaleFactor, 1.2);
    nh.param<int>("orb_lio/nLevels", nLevels, 8);
    nh.param<int>("orb_lio/fIniThFAST", fIniThFAST, 20);
    nh.param<int>("orb_lio/fMinThFAST", fMinThFAST, 7);

    mpORBextractor = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    cout << endl  << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;
}

void ORB_modify::ORB_feature(cv::Mat &im)
{
    cv::Mat mImGray = im;
    cv::Mat Image;

    (*mpORBextractor)(mImGray,cv::Mat(),mvKeys,mDescriptors);

    N = mvKeys.size();
    if(mvKeys.empty())
    {
        cout<<"no orb features?" <<endl;
        return;
    }
    // drawKeypoints(im, mvKeysUn, Image, cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DEFAULT);

    // cv::imshow("orb_feature", Image);
    // cv::imwrite("./result/orb_modify.png", Image);
}