

#include "VirtualKinect.h"


//////////////////////////////////////////////////////////////////////////
//
//                  VirtualKinect
//
//////////////////////////////////////////////////////////////////////////

void VirtualKinect::showError(const std::string err)
{
#if (defined WIN32 || defined _WIN32 || defined WINCE) // for Windows
    SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE),FOREGROUND_INTENSITY | FOREGROUND_RED);
    std::cerr << "Error: " << err << std::endl; 
    SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE),FOREGROUND_INTENSITY | FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE);
#else
    std::cout << "Error: " << err << std::endl;
#endif
}

void VirtualKinect::showVideoInfo(cv::VideoCapture& video)
{
#if (defined WIN32 || defined _WIN32 || defined WINCE) // for Windows
    SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE),FOREGROUND_INTENSITY | FOREGROUND_GREEN);
#endif

    std::cout << cv::format("frame count:%.0f, size (%.0f,%.0f), fps:%.2f, fourcc:",
        video.get(CV_CAP_PROP_FRAME_COUNT),
        video.get(CV_CAP_PROP_FRAME_WIDTH),
        video.get(CV_CAP_PROP_FRAME_HEIGHT),
        video.get(CV_CAP_PROP_FPS));

    int ex = static_cast<int>(video.get(CV_CAP_PROP_FOURCC));     // Get Codec Type- Int form
    char EXT[] = {(char)(ex & 0XFF) , (char)((ex & 0XFF00) >> 8),(char)((ex & 0XFF0000) >> 16),(char)((ex & 0XFF000000) >> 24), 0};
    std::cout << EXT << std::endl << std::endl;

#if (defined WIN32 || defined _WIN32 || defined WINCE) // for Windows
    SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE),FOREGROUND_INTENSITY | FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE);
#endif
}

std::string VirtualKinect::getCurrTimeString()
{
    time_t rawtime;
    struct tm *ts;
    char buf[80];

    time(&rawtime);
    ts = localtime(&rawtime);
    strftime(buf, sizeof(buf), "%Y%m%d%H%M%S", ts);

    return buf;
}


bool VirtualKinect::isFolderExist(const std::string folder, bool createFolder /*= false*/ )
{
    //////////////////////////////////////////////////////////////////////////
    //!!need <direct.h> and <io.h> included
    //////////////////////////////////////////////////////////////////////////
    bool RetVal = false;

    std::string fileName = folder;
    fileName += "\\*";

#if (defined WIN32 || defined _WIN32 || defined WINCE) // for Windows

    _finddata_t FileInfo;
    long Handle = _findfirst(fileName.c_str(), &FileInfo);	

    if ( Handle == -1L )
    {
        if ( createFolder )
        {
            if( 0 == _mkdir( folder.c_str() ) )
            {
                RetVal = true;
            }
        }
    }
    else
    {
        RetVal = true;
        _findclose(Handle);
    }

#else

    if( NULL == opendir( folder.c_str() ) )
    {
        if (createFolder)
        {
            if( 0 == mkdir( folder.c_str(), 0775) ) 
            {
                RetVal = true;
            }
        }
    }
    else
    {
        RetVal = true;
    }

#endif
    return RetVal;
}

bool VirtualKinect::isFileReadable(const std::string file)
{
/*    //////////////////////////////////////////////////////////////////////////
    //!!need <io.h> included
    //////////////////////////////////////////////////////////////////////////
    bool RetVal = false;

    _finddata_t FileInfo;
    long Handle = _findfirst(file.c_str(), &FileInfo);	

    if (Handle == -1L)
    {
        RetVal = false;
        _findclose(Handle);
    }
    else
    {
        RetVal = true;
        _findclose(Handle);
    }

    return RetVal;*/

    FILE *fp;
    fp = fopen(file.c_str(), "r");
    if (fp)
    {
        fclose(fp);
        return true;
    }
    else
        return false;
}

bool VirtualKinect::depth_16uc1_8uc3(const cv::Mat& depth, cv::Mat& depthImg)
{
    if (depth.type() != CV_16UC1)
        return false;

    depthImg = cv::Mat::zeros(depth.size(), CV_8UC3);

    cv::MatConstIterator_<ushort> itDepth = depth.begin<ushort>();
    cv::MatConstIterator_<ushort> itEnd = depth.end<ushort>();
    cv::MatIterator_<cv::Vec3b> itImg = depthImg.begin<cv::Vec3b>();

    for (;itDepth != itEnd; ++itDepth, ++itImg)
    {
        ushort dval = *itDepth;
        uchar ch1 = dval & 0x00FF;
        uchar ch2 = dval>>8;
        *itImg = cv::Vec3b(ch1, ch2, 0);
    }

    return true;
}

bool VirtualKinect::depth_8uc3_16uc1(const cv::Mat& depthImg, cv::Mat& depth)
{
    if (depthImg.type() != CV_8UC3)
        return false;

    depth = cv::Mat::zeros(depthImg.size(), CV_16UC1);

    cv::MatConstIterator_<cv::Vec3b> itImg = depthImg.begin<cv::Vec3b>();
    cv::MatConstIterator_<cv::Vec3b> itEnd = depthImg.end<cv::Vec3b>();
    cv::MatIterator_<ushort> itDepth = depth.begin<ushort>();

    for (;itImg != itEnd; ++itImg, ++itDepth)
    {
        cv::Vec3b ival = *itImg;
        ushort v1 = ival[0];
        ushort v2 = ival[1]<<8;
        ushort dval = v1+v2;
        *itDepth = dval;
    }

    return true;
}


bool VirtualKinect::averagingDepthMaps(std::vector<cv::Mat>& multiDepths, cv::Mat& depth)
{
    bool result = false;

    cv::Mat depthSum = cv::Mat::zeros(multiDepths[0].size(), CV_32FC1);
    cv::Mat nonZero = cv::Mat::zeros(multiDepths[0].size(), CV_32FC1);

    for (size_t ii = 1; ii < multiDepths.size(); ii++) 
    {
        cv::MatConstIterator_<ushort> pSrc = multiDepths[ii].begin<ushort>(), pSrcEnd = multiDepths[ii].end<ushort>();
        cv::MatIterator_<float> pDst = depthSum.begin<float>();
        cv::MatIterator_<float> pCount = nonZero.begin<float>();

        for (; pSrc != pSrcEnd; pSrc++, pDst++, pCount++) 
        {
            *pDst += *pSrc;
            *pCount += (*pSrc > 0) ? 1 : 0;
        }
    }

    cv::MatIterator_<float> pDst = depthSum.begin<float>(), pDstEnd = depthSum.end<float>();
    cv::MatIterator_<float> pCount = nonZero.begin<float>();

    for (; pDst != pDstEnd; pDst++, pCount++) 
    {
        *pDst = (*pCount > 0) ? (*pDst / *pCount) : 0;
    }

    depthSum.convertTo(depth, CV_16UC1);

    return result;
}


void VirtualKinect::mapProjectiveToRealWorld(cv::Point3f& src, cv::Point3f& dst)
{
    float fXToZ = std::tan(DEFAULT_DEPTH_FOV_H/2)*2;
    float fYToZ = std::tan(DEFAULT_DEPTH_FOV_V/2)*2;

    float xn = src.x / 640 - 0.5;
    float yn = 0.5 - src.y / 480;

    float unitFactor = 0.001f; // unit: meter

    dst.x = unitFactor * xn * src.z * fXToZ;
    dst.y = unitFactor * yn * src.z * fYToZ;
    dst.z = unitFactor * src.z;
}

bool VirtualKinect::generatePointClouds(const cv::Mat& depth, cv::Mat& points)
{
    if( depth.empty() || (depth.type() != CV_16UC1 && depth.type() != CV_16SC1) )
        return false;

    points.create(depth.size(), CV_32FC3);

    int cols = depth.cols, rows = depth.rows;
    float co = (cols-1)/2.0, ro = (rows-1)/2.0;
    float scaleFactor = 0.0021f;
    int minDistance = -10;
    float unitFactor = 0.001f; // unit: meter

    for(int r=0; r<rows; r++)
    {
        cv::Point3f* cloud_ptr = (cv::Point3f*)points.ptr(r);
        const ushort* depth_ptr = (const ushort*)depth.ptr(r);
        for(int c=0; c<cols; c++)
        {

            if (depth_ptr[c] > 0)
            {
                float z = depth_ptr[c]; 
                cv::Point3f src(c,r,z);
                //mapProjectiveToRealWorld(src, cloud_ptr[c]);
                cloud_ptr[c].x = unitFactor * (c - co) * (z + minDistance) * scaleFactor;
                cloud_ptr[c].y = unitFactor * (ro - r) * (z + minDistance) * scaleFactor;
                cloud_ptr[c].z = unitFactor * z;
            } 
            else
            {
                cloud_ptr[c].x = 0;
                cloud_ptr[c].y = 0;
                cloud_ptr[c].z = 0;
            }
        }
    }

    return true;
}

//////////////////////////////////////////////////////////////////////////
//
//                  VirtualKinectWriter
//
//////////////////////////////////////////////////////////////////////////

bool VirtualKinectWriter::open(std::string dataFolder, VK_DEPTH_FILE_TYPE typeDepth, VK_COLOR_FILE_TYPE typeColor)
{
    dataFolder_ = dataFolder + cv::format("/data_%s/",tCreated_.c_str());
    depthFileType_ = typeDepth;
    colorFileType_ = typeColor;

    bOpened_ = isFolderExist(dataFolder_, true);

    if (!ofs_files_.is_open())
        ofs_files_.open((dataFolder_+"/vkDatas.txt").c_str(), std::ios::out);

    bOpened_ = ofs_files_.is_open();

    return bOpened_;
}

bool VirtualKinectWriter::write(cv::Mat& depth)
{
    cv::Mat image = cv::Mat();
    return write(depth, image);
}

bool VirtualKinectWriter::write(cv::Mat& depth, cv::Mat& image /*= cv::Mat()*/)
{
    bool result = false;

    if (!depth.empty() && depth.type() == CV_16UC1)
    {
        if (depthFileType_ == VK_DEPTH_FILE_VIDEO)
        {
            if (depthWriter_.isOpened() == false)
            {
                std::string avifile = dataFolder_+"depth.avi";
                result = depthWriter_.open(avifile, CV_FOURCC('D','I','B',' '), 30, depth.size());

                if (depthWriter_.isOpened())
                    ofs_files_ << avifile << std::endl;
            }

            if (depthWriter_.isOpened())
            {
                cv::Mat depth_8uc3;
                if (depth_16uc1_8uc3(depth, depth_8uc3))
                {
                    depthWriter_ << depth_8uc3;
                    frameCount_++;
                    result = true;
                }
                else
                    return false;
            }
        } 
        else if (depthFileType_ == VK_DEPTH_FILE_IMAGE)
        {
            std::string imgfile = dataFolder_+cv::format("depth_%05d.png",frameCount_);
            cv::imwrite(imgfile, depth);
            ofs_files_ << imgfile << std::endl;

            result = true;
            //cv::Mat depth_8uc3;
            //if (depth_16uc1_8uc3(depth, depth_8uc3))
            //{
            //    std::string imgfile = dataFolder_+cv::format("depth_%05d.png",frameCount_);
            //    cv::imwrite(imgfile, depth_8uc3);
            //    ofs_files_ << imgfile << std::endl;

            //    result = true;
            //}
            //else
            //    return false;
        }
    }

    if (!image.empty() && image.type() == CV_8UC3 && image.size() == depth.size())
    {
        if (colorFileType_ == VK_COLOR_FILE_VIDEO)
        {
            if (imageWriter_.isOpened() == false)
            {
                std::string avifile = dataFolder_+"color.avi";

                // OpenCV 的 ffmpeg 在读取多个视频时，若视频格式不一致、且其中至少一个为无损压缩格式时
                // 就容易在读取过程中出错，因此在记录kinect数据时，如果深度图采用视频模式来记录
                // 则彩色图像也必须用 同样的编码格式 来记录
                if (depthFileType_ == VK_DEPTH_FILE_VIDEO)
                    imageWriter_.open(avifile, CV_FOURCC('D','I','B',' '), 30, image.size());
                else
                    imageWriter_.open(avifile, CV_FOURCC('X','V','I','D'), 30, image.size());

                if (imageWriter_.isOpened())
                    ofs_files_ << avifile << std::endl;
            }

            if (imageWriter_.isOpened())
            {
                imageWriter_ << image;

                result = true;
            }
        }
        else if (colorFileType_ == VK_COLOR_FILE_IMAGE)
        {
            std::string imgfile = dataFolder_+cv::format("color_%05d.png", frameCount_);
            cv::imwrite(imgfile, image);
            ofs_files_ << imgfile << std::endl;

            result = true;
        }
    }

    if (result)
        frameCount_++;

    return (result);
}

//////////////////////////////////////////////////////////////////////////
//
//                  VirtualKinectCapture
//
//////////////////////////////////////////////////////////////////////////

int VirtualKinectCapture::findSubString(std::string S, std::string T, int pos /*= 0*/) 
{
    int slen = S.length(), tlen = T.length();
    int i,start;  

    if(tlen == 0)
        return 0;

    start = -1;
    i = pos;

    while(i <= (slen-tlen))
    {
        std::string s = S.substr(i,tlen);
        if (s.compare(T) == 0)
        {
            start = i;
            break;
        }
        i++;
    }

    return (start);
}

bool VirtualKinectCapture::parseFileList(const std::string vkDatas)
{
    std::ifstream fs(vkDatas.c_str());

    if (fs.is_open())
    {           
        do 
        {
            std::string line;
            std::getline(fs, line);

            if (line.empty())
                continue;

            int found = -1;
            found = line.find_first_not_of(" │:");
            if (found>=0)
            {
                line = line.substr(found);
            }

            if (line[line.length()-1] == '\n')
                line = line.substr(0, line.length()-1);

            found = findSubString(line, "depth");
            if (found>=0 && isFileReadable(line))
            {
                found = line.find_last_of('.');
                if (found>=0)
                {
                    std::string fmt = line.substr(found+1, line.length());
                    std::transform(fmt.begin(), fmt.end(), fmt.begin(), ::tolower);

                    VK_DEPTH_FILE_TYPE type = VK_DEPTH_FILE_UNKOWN; 
                    if (fmt.compare("avi") == 0)
                    {
                        type = VK_DEPTH_FILE_VIDEO;
                    }
                    else if (fmt.compare("png") == 0)
                    {
                        type = VK_DEPTH_FILE_IMAGE;
                    }

                    if (type != VK_DEPTH_FILE_UNKOWN)
                    {
                        if (depthFileType_ == VK_DEPTH_FILE_UNKOWN)
                            depthFileType_ = type;

                        // ensure that the format of depth file is unique
                        if (type == depthFileType_) 
                            depthFile_.push_back(line);
                    }
                }
            }

            found = findSubString(line, "color");
            if (found>=0 && isFileReadable(line))
            {
                found = line.find_last_of('.');
                if (found>=0)
                {
                    std::string fmt = line.substr(found+1, line.length());
                    std::transform(fmt.begin(), fmt.end(), fmt.begin(), ::tolower);

                    VK_DEPTH_FILE_TYPE type = VK_COLOR_FILE_UNKOWN;
                    if (fmt.compare("avi") == 0)
                    {
                        type = VK_COLOR_FILE_VIDEO;
                    }
                    else if (fmt.compare("png") == 0)
                    {
                        type = VK_COLOR_FILE_IMAGE;
                    }

                    if (type != VK_COLOR_FILE_UNKOWN)
                    {
                        if (colorFileType_ == VK_COLOR_FILE_UNKOWN)
                            colorFileType_ = type;

                        // ensure that the format of depth file is unique
                        if (type == colorFileType_)
                            colorFile_.push_back(line);
                    }
                }
            }

        } while (!fs.eof());
    }
    else
    {
        showError( cv::format("Failed to open %s", vkDatas.c_str()) );
    }

    if (!depthFile_.empty() && !colorFile_.empty())
    {
        //std::ofstream ofs(vkDatas.c_str(), std::ios::out);
        //if (ofs.is_open())
        //{
        //    for (int i = 0; i < depthFile_.size(); i++)
        //        ofs << depthFile_[i] << std::endl;

        //    for (int j = 0; j < colorFile_.size(); j++)
        //        ofs << colorFile_[j] << std::endl;

        //    ofs.close();
        //}

        return true;
    }
    else
        return false;
}

bool VirtualKinectCapture::open(std::string filelist)
{
    if (!bOpened_)
    {
        bOpened_ = parseFileList(filelist);

        if (bOpened_)
        {
            if ((depthFileType_ == VK_DEPTH_FILE_VIDEO) && depthCapture_.open(depthFile_[0]))
            {
                showVideoInfo(depthCapture_);

                frameCount_ = depthCapture_.get(CV_CAP_PROP_FRAME_COUNT);
            }
            else if (depthFileType_ == VK_DEPTH_FILE_IMAGE)
            {
                frameCount_ = depthFile_.size();
            }
            bOpened_ = frameCount_ != 0;

            if ((colorFileType_ == VK_COLOR_FILE_VIDEO) && colorCapture_.open(colorFile_[0]))
            {
                bOpened_ = colorCapture_.get(CV_CAP_PROP_FRAME_COUNT) == frameCount_;

                if (!bOpened_)
                    std::cerr << "Error: frame count of BGR images is not equal to that of depth maps !\n";
                else
                {
                    showVideoInfo(colorCapture_);

                    bOpened_ = colorCapture_.get(CV_CAP_PROP_FOURCC) == depthCapture_.get(CV_CAP_PROP_FOURCC);
                    if (!bOpened_)
                        std::cerr << "The codecs of image video and depth video must be the same!\n";
                }

            }
            else if (colorFileType_ = VK_COLOR_FILE_IMAGE)
            {
                bOpened_ = colorFile_.size() == frameCount_;
                if (!bOpened_)
                    std::cerr << "Error: frame count of BGR images is not equal to that of depth maps !\n";
            }
        }
    }

    return (bOpened_);
}

void VirtualKinectCapture::release()
{
    dID_ = framePos_ = frameCount_ = 0;
    bOpened_ = false;
    depthFileType_ = VK_DEPTH_FILE_UNKOWN;
    colorFile_.clear();
    depthFile_.clear();
    image_.release();
    depth_.release();
    point_.release();
    colorCapture_.release();
    depthCapture_.release();
}

bool VirtualKinectCapture::grab()
{
    return grab(-1);
}

bool VirtualKinectCapture::grab(int frameID)
{
    bool result = false;
    int fid = -1;

    if (frameID < 0)
    {
        fid = dID_;
        dID_++;
        if (dID_ >= frameCount_)
            dID_ = 0;
    }
    else
        fid = frameID % frameCount_;

    if ((depthFileType_ == VK_DEPTH_FILE_VIDEO) && depthCapture_.isOpened())
    {
        cv::Mat depth_8uc3;
        result = depthCapture_.read(depth_8uc3);

        if (result)
        {
            result = depth_8uc3_16uc1(depth_8uc3, depth_);
        }
    }
    else if (depthFileType_ == VK_DEPTH_FILE_IMAGE)
    {
        //std::cout << "Depth File: " << depthFile_[fid] << std::endl;
        cv::Mat depth_read = cv::imread(depthFile_[fid], -1);
        if (!depth_read.empty())
        {
            if (depth_read.type() == CV_8UC3)
                result = depth_8uc3_16uc1(depth_read, depth_);
            else if (depth_read.type() == CV_16UC1) 
            {
                depth_ = depth_read.clone();
                result = true;
            }
        }
    } 

    if ((colorFileType_ == VK_COLOR_FILE_VIDEO) && colorCapture_.isOpened())
    {
        result = colorCapture_.read(image_);
    }
    else if (colorFileType_ == VK_COLOR_FILE_IMAGE)
    {
        //std::cout << "Color File: " << colorFile_[fid] << std::endl;
        image_ = cv::imread(colorFile_[fid]);

        result = (image_.empty() == false);
    }

    return (result);
}

bool VirtualKinectCapture::retrieve(cv::Mat& frame, int flag)
{
    if ((flag == VK_CAP_TYPE_DEPTH || flag == VK_CAP_TYPE_POINT) && !depth_.empty())
    {
        if (flag == VK_CAP_TYPE_POINT)
        {
            generatePointClouds(depth_, frame);
        }
        else
        {
            depth_.copyTo(frame);
        }
    }
    else if ((flag == VK_CAP_TYPE_IMAGE) && !image_.empty())
    {
        image_.copyTo(frame);
    }
    else
        frame.release();

    return !frame.empty();
}

