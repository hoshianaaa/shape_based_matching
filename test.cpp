#include "line2Dup.h"
#include <memory>
#include <iostream>
#include <assert.h>
#include <chrono>

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Polygon.h"
#include "std_msgs/Int32.h"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

const double PI=3.141592653589793;

using namespace std;
using namespace cv;

const bool DEBUG = false;

std::string name_space_ = "";

static std::string prefix = "/shape_based_matching/test/";

int temp_img_width = 50;
int temp_img_height = 100;

double polygon_occupancy(std::vector<cv::Point> ps, cv::Mat image)
{
  double per = 1.0;
  cv::Mat img = image.clone();

  cv::Mat mask = cv::Mat::zeros(img.rows, img.cols, CV_8UC1);
  cv::fillConvexPoly(mask, ps, cv::Scalar(255));
  int polygon_pix_num = cv::countNonZero(mask);
  std::cout << "polygon pix num :" << polygon_pix_num << std::endl;
  cv::bitwise_and(img, mask, img);
  int in_polygon_pix_num = cv::countNonZero(img);
  std::cout << "in polygon pix num :" << in_polygon_pix_num << std::endl;
  per = (double)in_polygon_pix_num / polygon_pix_num;
  //cv::imshow("mask", mask);
  //cv::imshow("masked_img", img);
  return per;
}

cv::Point rotate(cv::Point p, double deg)
{
  double rad = PI * deg / 180;
  cv::Point res;
  res.x = int( std::cos(rad) * p.x - std::sin(rad) * p.y);
  res.y = int (std::sin(rad) * p.x + std::cos(rad) * p.y);
  return res;
}

class Timer
{
public:
    Timer() : beg_(clock_::now()) {}
    void reset() { beg_ = clock_::now(); }
    double elapsed() const {
        return std::chrono::duration_cast<second_>
            (clock_::now() - beg_).count(); }
    void out(std::string message = ""){
        double t = elapsed();
        std::cout << message << "\nelasped time:" << t << "s" << std::endl;
        reset();
    }
private:
    typedef std::chrono::high_resolution_clock clock_;
    typedef std::chrono::duration<double, std::ratio<1> > second_;
    std::chrono::time_point<clock_> beg_;
};
// NMS, got from cv::dnn so we don't need opencv contrib
// just collapse it
namespace  cv_dnn {
namespace
{

template <typename T>
static inline bool SortScorePairDescend(const std::pair<float, T>& pair1,
                          const std::pair<float, T>& pair2)
{
    return pair1.first > pair2.first;
}

} // namespace

inline void GetMaxScoreIndex(const std::vector<float>& scores, const float threshold, const int top_k,
                      std::vector<std::pair<float, int> >& score_index_vec)
{
    for (size_t i = 0; i < scores.size(); ++i)
    {
        if (scores[i] > threshold)
        {
            score_index_vec.push_back(std::make_pair(scores[i], i));
        }
    }
    std::stable_sort(score_index_vec.begin(), score_index_vec.end(),
                     SortScorePairDescend<int>);
    if (top_k > 0 && top_k < (int)score_index_vec.size())
    {
        score_index_vec.resize(top_k);
    }
}

template <typename BoxType>
inline void NMSFast_(const std::vector<BoxType>& bboxes,
      const std::vector<float>& scores, const float score_threshold,
      const float nms_threshold, const float eta, const int top_k,
      std::vector<int>& indices, float (*computeOverlap)(const BoxType&, const BoxType&))
{
    CV_Assert(bboxes.size() == scores.size());
    std::vector<std::pair<float, int> > score_index_vec;
    GetMaxScoreIndex(scores, score_threshold, top_k, score_index_vec);

    // Do nms.
    float adaptive_threshold = nms_threshold;
    indices.clear();
    for (size_t i = 0; i < score_index_vec.size(); ++i) {
        const int idx = score_index_vec[i].second;
        bool keep = true;
        for (int k = 0; k < (int)indices.size() && keep; ++k) {
            const int kept_idx = indices[k];
            float overlap = computeOverlap(bboxes[idx], bboxes[kept_idx]);
            keep = overlap <= adaptive_threshold;
        }
        if (keep)
            indices.push_back(idx);
        if (keep && eta < 1 && adaptive_threshold > 0.5) {
          adaptive_threshold *= eta;
        }
    }
}


// copied from opencv 3.4, not exist in 3.0
template<typename _Tp> static inline
double jaccardDistance__(const Rect_<_Tp>& a, const Rect_<_Tp>& b) {
    _Tp Aa = a.area();
    _Tp Ab = b.area();

    if ((Aa + Ab) <= std::numeric_limits<_Tp>::epsilon()) {
        // jaccard_index = 1 -> distance = 0
        return 0.0;
    }

    double Aab = (a & b).area();
    // distance = 1 - jaccard_index
    return 1.0 - Aab / (Aa + Ab - Aab);
}

template <typename T>
static inline float rectOverlap(const T& a, const T& b)
{
    return 1.f - static_cast<float>(jaccardDistance__(a, b));
}

void NMSBoxes(const std::vector<Rect>& bboxes, const std::vector<float>& scores,
                          const float score_threshold, const float nms_threshold,
                          std::vector<int>& indices, const float eta=1, const int top_k=0)
{
    NMSFast_(bboxes, scores, score_threshold, nms_threshold, eta, top_k, indices, rectOverlap);
}

}

void scale_test(string mode = "test"){
    int num_feature = 150;

    // feature numbers(how many ori in one templates?)
    // two pyramids, lower pyramid(more pixels) in stride 4, lower in stride 8
    line2Dup::Detector detector(num_feature, {4, 8});

//    mode = "test";
    if(mode == "train"){
        Mat img = cv::imread(prefix+"case0/templ/circle.png");
        assert(!img.empty() && "check your img path");
        shape_based_matching::shapeInfo_producer shapes(img);

        shapes.scale_range = {0.1f, 1};
        shapes.scale_step = 0.01f;
        shapes.produce_infos();

        std::vector<shape_based_matching::shapeInfo_producer::Info> infos_have_templ;
        string class_id = "circle";
        for(auto& info: shapes.infos){

            // template img, id, mask,
            //feature numbers(missing it means using the detector initial num)
            int templ_id = detector.addTemplate(shapes.src_of(info), class_id, shapes.mask_of(info),
                                                int(num_feature*info.scale));
            //std::cout << "templ_id: " << templ_id << std::endl;

            // may fail when asking for too many feature_nums for small training img
            if(templ_id != -1){  // only record info when we successfully add template
                infos_have_templ.push_back(info);
            }
        }

        // save templates
        detector.writeClasses(prefix+"case0/%s_templ.yaml");

        // save infos,
        // in this simple case infos are not used
        shapes.save_infos(infos_have_templ, prefix + "case0/circle_info.yaml");
        std::cout << "train end" << std::endl << std::endl;

    }else if(mode=="test"){
        std::vector<std::string> ids;

        // read templates
        ids.push_back("circle");
        detector.readClasses(ids, prefix+"case0/%s_templ.yaml");

        Mat test_img = imread(prefix+"case0/1.jpg");
        assert(!test_img.empty() && "check your img path");

        // make the img having 32*n width & height
        // at least 16*n here for two pyrimads with strides 4 8
        int stride = 32;
        int n = test_img.rows/stride;
        int m = test_img.cols/stride;
        Rect roi(0, 0, stride*m , stride*n);
        Mat img = test_img(roi).clone();
        assert(img.isContinuous());

        Timer timer;
        // match, img, min socre, ids
        auto matches = detector.match(img, 90, ids);
        // one output match:
        // x: top left x
        // y: top left y
        // template_id: used to find templates
        // similarity: scores, 100 is best
        timer.out();

        std::cout << "matches.size(): " << matches.size() << std::endl;
        size_t top5 = 5;
        if(top5>matches.size()) top5=matches.size();
        for(size_t i=0; i<top5; i++){
            auto match = matches[i];
            auto templ = detector.getTemplates("circle",
                                               match.template_id);
            // template:
            // nums: num_pyramids * num_modality (modality, depth or RGB, always 1 here)
            // template[0]: lowest pyrimad(more pixels)
            // template[0].width: actual width of the matched template
            // template[0].tl_x / tl_y: topleft corner when cropping templ during training
            // In this case, we can regard width/2 = radius
            int x =  templ[0].width/2 + match.x;
            int y = templ[0].height/2 + match.y;
            int r = templ[0].width/2;
            Scalar color(255, rand()%255, rand()%255);

            cv::putText(img, to_string(int(round(match.similarity))),
                        Point(match.x+r-10, match.y-3), FONT_HERSHEY_PLAIN, 2, color);
            cv::circle(img, {x, y}, r, color, 2);
        }

        //imshow("img", img);
        waitKey(10);

        std::cout << "test end" << std::endl << std::endl;
    }
}

bool angle_train(bool use_rot, Mat image){
   try{
    std::cout << "start train!"<< std::endl;
    line2Dup::Detector detector(128, {4, 8});

       //Mat img = imread(prefix+"case1/train.png",CV_LOAD_IMAGE_GRAYSCALE);
        Mat img = image;
        assert(!img.empty() && "check your img path");
	//imshow("test", img);
	//waitKey(0);

        img = img.clone();
        Mat mask = Mat(img.size(), CV_8UC1, {255});
	//imshow("test2", img);
	//waitKey(0);

        // padding to avoid rotating out
        int padding = 0;
        cv::Mat padded_img = cv::Mat(img.rows + 2*padding, img.cols + 2*padding, img.type(), cv::Scalar::all(0));
        img.copyTo(padded_img(Rect(padding, padding, img.cols, img.rows)));

        cv::Mat padded_mask = cv::Mat(mask.rows + 2*padding, mask.cols + 2*padding, mask.type(), cv::Scalar::all(0));
        mask.copyTo(padded_mask(Rect(padding, padding, img.cols, img.rows)));

        shape_based_matching::shapeInfo_producer shapes(padded_img, padded_mask);
        shapes.angle_range = {0, 360};
        shapes.angle_step = 1;

        shapes.scale_range = {1}; // support just one
        shapes.produce_infos();
        std::vector<shape_based_matching::shapeInfo_producer::Info> infos_have_templ;
        string class_id = "test";

        bool is_first = true;

        // for other scales you want to re-extract points: 
        // set shapes.scale_range then produce_infos; set is_first = false;

        int first_id = 0;
        float first_angle = 0;
        for(auto& info: shapes.infos){
            Mat to_show = shapes.src_of(info);

            //std::cout << "\ninfo.angle: " << info.angle << std::endl;
            int templ_id;

            if(is_first){
                templ_id = detector.addTemplate(shapes.src_of(info), class_id, shapes.mask_of(info));
                first_id = templ_id;
                first_angle = info.angle;

                if(use_rot) is_first = false;
            }else{
                templ_id = detector.addTemplate_rotate(class_id, first_id,
                                                       info.angle-first_angle,
                                                {shapes.src.cols/2.0f, shapes.src.rows/2.0f});
            }

            auto templ = detector.getTemplates("test", templ_id);
            for(int i=0; i<templ[0].features.size(); i++){
                auto feat = templ[0].features[i];
                cv::circle(to_show, {feat.x+templ[0].tl_x, feat.y+templ[0].tl_y}, 3, {0, 0, 255}, -1);
            }
            
            // will be faster if not showing this
     //       imshow("train", to_show);
     //      waitKey(1);

            //std::cout << "templ_id: " << templ_id << std::endl;
            if(templ_id != -1){
                infos_have_templ.push_back(info);
            }
        }
        detector.writeClasses(prefix+"case1/%s_templ.yaml");
        shapes.save_infos(infos_have_templ, prefix + "case1/test_info.yaml");
        std::cout << "train end" << std::endl << std::endl;
	return 1;
     }
     catch(...)
     {
       return 0;
     }
}



bool angle_test( bool use_rot, cv::Mat image, double &x, double &y, double &angle, std::vector<double> &edges_x, std::vector<double> &edges_y){

  try{
      
        if ((image.cols == 0) || (image.rows == 0))
          return 0;

	//std::cout << "debug1" << std::endl;
    line2Dup::Detector detector(128, {4, 8});

        std::vector<std::string> ids;
        ids.push_back("test");
        detector.readClasses(ids, prefix+"case1/%s_templ.yaml");

        int id = 0;
        auto templ = detector.getTemplates("test", id);

        edges_x.clear();
        edges_y.clear();

        double sum_x = 0;
        double sum_y = 0;

        int size = templ[0].features.size();

        for(int i=0; i<size; i++){
            auto feat = templ[0].features[i];
            sum_x += feat.x;
            sum_y += feat.y;
            //edges_x.push_back(feat.x);
            //edges_y.push_back(feat.y);
        }

        double center_x = sum_x / size;
        double center_y = sum_y / size;

        for(int i=0; i<size; i++){
            auto feat = templ[0].features[i];
            edges_x.push_back(feat.x - center_x);
            edges_y.push_back(feat.y - center_y);
        }

        // angle & scale are saved here, fetched by match id
        auto infos = shape_based_matching::shapeInfo_producer::load_infos(prefix + "case1/test_info.yaml");

        Mat test_img = image;
        assert(!test_img.empty() && "check your img path");

        int padding = 100;
        cv::Mat padded_img = cv::Mat(test_img.rows + 2*padding,
                                     test_img.cols + 2*padding, test_img.type(), cv::Scalar::all(0));
        test_img.copyTo(padded_img(Rect(padding, padding, test_img.cols, test_img.rows)));

        int stride = 16;
        int n = padded_img.rows/stride;
        int m = padded_img.cols/stride;
        Rect roi(0, 0, stride*m , stride*n);
        Mat img = padded_img(roi).clone();
        assert(img.isContinuous());

//        cvtColor(img, img, CV_BGR2GRAY);

        //std::cout << "test img size: " << img.rows * img.cols << std::endl << std::endl;

        Timer timer;
        auto matches = detector.match(img, 30, ids); // image, threshold(0 ~ 100?)
        timer.out();

        if(img.channels() == 1) cvtColor(img, img, CV_GRAY2BGR);

        //std::cout << "matches.size(): " << matches.size() << std::endl;

        if (matches.size() == 0)
          return 0;
        
        size_t top5 = 1;

        double result_x, result_y, result_ang;
        if(top5>matches.size()) top5=matches.size();
        for(size_t i=0; i<top5; i++){
            auto match = matches[i];
            auto templ = detector.getTemplates("test",
                                               match.template_id);

            // 270 is width of template image
            // 100 is padding when training
            // tl_x/y: template croping topleft corner when training

            float r_scaled = temp_img_width/2.0f*infos[match.template_id].scale;

            // scaling won't affect this, because it has been determined by warpAffine
            // cv::warpAffine(src, dst, rot_mat, src.size()); last param
            float train_img_half_width = temp_img_width/2.0f;
            float train_img_half_height = temp_img_height/2.0f;

            // center x,y of train_img in test img
            //float x =  match.x - templ[0].tl_x + train_img_half_width;
            float x =  match.x - templ[0].tl_x + templ[0].width;
            //float y =  match.y - templ[0].tl_y + train_img_half_height;
            float y =  match.y - templ[0].tl_y + templ[0].height;

            cv::Vec3b randColor;
            randColor[0] = rand()%155 + 100;
            randColor[1] = rand()%155 + 100;
            randColor[2] = rand()%155 + 100;

            double sum_x = 0;
            double sum_y = 0;
            for(int i=0; i<templ[0].features.size(); i++){
                auto feat = templ[0].features[i];
                cv::circle(img, {feat.x+match.x, feat.y+match.y}, 3, randColor, -1);
                sum_x += (feat.x+match.x);
                sum_y += (feat.y+match.y);

            }

            double center_x = sum_x / templ[0].features.size();
            double center_y = sum_y / templ[0].features.size();

            if (i == 0)
            {
              result_x = center_x - padding;
              result_y = center_y - padding;
              result_ang = match.template_id;
            }

            cv::putText(img, to_string(int(round(match.similarity))),
                        Point(match.x+r_scaled-10, match.y-3), FONT_HERSHEY_PLAIN, 2, randColor);
            cv::drawMarker(img, cv::Point(int(center_x),int(center_y)), cv::Vec3b(0,0,200), cv::MARKER_CROSS);


            /*
            cv::RotatedRect rotatedRectangle({x, y}, {2*r_scaled, 2*r_scaled}, -infos[match.template_id].angle);

            cv::Point2f vertices[4];
            rotatedRectangle.points(vertices);
            for(int i=0; i<4; i++){
                int next = (i+1==4) ? 0 : (i+1);
                cv::line(img, vertices[i], vertices[next], randColor, 2);
            }
            */

            //std::cout << "\nmatch.template_id: " << match.template_id << std::endl;
            //std::cout << "match.similarity: " << match.similarity << std::endl;
        }

        if (DEBUG)
        {
          imshow("img", img);
          waitKey(1);
        }

        //std::cout << "test end" << std::endl << std::endl;

        x = result_x;
        y = result_y;
        angle = result_ang;
        return true;
  }
  catch(...)
  {
    return 0;
  }
}

void noise_test(string mode = "test"){
    line2Dup::Detector detector(30, {4, 8});

    if(mode == "train"){
        Mat img = imread(prefix+"case2/train.png");
        assert(!img.empty() && "check your img path");
        Mat mask = Mat(img.size(), CV_8UC1, {255});

        shape_based_matching::shapeInfo_producer shapes(img, mask);
        shapes.angle_range = {0, 360};
        shapes.angle_step = 1;
        shapes.produce_infos();
        std::vector<shape_based_matching::shapeInfo_producer::Info> infos_have_templ;
        string class_id = "test";
        for(auto& info: shapes.infos){
            imshow("train", shapes.src_of(info));
            waitKey(1);

            std::cout << "\ninfo.angle: " << info.angle << std::endl;
            int templ_id = detector.addTemplate(shapes.src_of(info), class_id, shapes.mask_of(info));
            //std::cout << "templ_id: " << templ_id << std::endl;
            if(templ_id != -1){
                infos_have_templ.push_back(info);
            }
        }
        detector.writeClasses(prefix+"case2/%s_templ.yaml");
        shapes.save_infos(infos_have_templ, prefix + "case2/test_info.yaml");
        std::cout << "train end" << std::endl << std::endl;
    }else if(mode=="test"){
        std::vector<std::string> ids;
        ids.push_back("test");
        detector.readClasses(ids, prefix+"case2/%s_templ.yaml");

        Mat test_img = imread(prefix+"case2/test.png");
        assert(!test_img.empty() && "check your img path");

        // cvtColor(test_img, test_img, CV_BGR2GRAY);

        int stride = 16;
        int n = test_img.rows/stride;
        int m = test_img.cols/stride;
        Rect roi(0, 0, stride*m , stride*n);

        test_img = test_img(roi).clone();

        Timer timer;
        auto matches = detector.match(test_img, 90, ids);
        timer.out();

        std::cout << "matches.size(): " << matches.size() << std::endl;
        size_t top5 = 500;
        if(top5>matches.size()) top5=matches.size();

        vector<Rect> boxes;
        vector<float> scores;
        vector<int> idxs;
        for(auto match: matches){
            Rect box;
            box.x = match.x;
            box.y = match.y;

            auto templ = detector.getTemplates("test",
                                               match.template_id);

            box.width = templ[0].width;
            box.height = templ[0].height;
            boxes.push_back(box);
            scores.push_back(match.similarity);
        }
        cv_dnn::NMSBoxes(boxes, scores, 0.1, 0.2f, idxs);

        for(auto idx: idxs){
            auto match = matches[idx];
            auto templ = detector.getTemplates("test",
                                               match.template_id);

            int x =  templ[0].width + match.x;
            int y = templ[0].height + match.y;
            int r = templ[0].width/2;
            cv::Vec3b randColor;
            randColor[0] = rand()%155 + 100;
            randColor[1] = rand()%155 + 100;
            randColor[2] = rand()%155 + 100;

            for(int i=0; i<templ[0].features.size(); i++){
                auto feat = templ[0].features[i];
                cv::circle(test_img, {feat.x+match.x, feat.y+match.y}, 2, randColor, -1);
            }

            cv::putText(test_img, to_string(int(round(match.similarity))),
                        Point(match.x+r-10, match.y-3), FONT_HERSHEY_PLAIN, 2, randColor);
            cv::rectangle(test_img, {match.x, match.y}, {x, y}, randColor, 2);

            std::cout << "\nmatch.template_id: " << match.template_id << std::endl;
            std::cout << "match.similarity: " << match.similarity << std::endl;
        }

        imshow("img", test_img);
        waitKey(0);

        std::cout << "test end" << std::endl << std::endl;
    }
}

void MIPP_test(){
    std::cout << "MIPP tests" << std::endl;
    std::cout << "----------" << std::endl << std::endl;

    std::cout << "Instr. type:       " << mipp::InstructionType                  << std::endl;
    std::cout << "Instr. full type:  " << mipp::InstructionFullType              << std::endl;
    std::cout << "Instr. version:    " << mipp::InstructionVersion               << std::endl;
    std::cout << "Instr. size:       " << mipp::RegisterSizeBit       << " bits" << std::endl;
    std::cout << "Instr. lanes:      " << mipp::Lanes                            << std::endl;
    std::cout << "64-bit support:    " << (mipp::Support64Bit    ? "yes" : "no") << std::endl;
    std::cout << "Byte/word support: " << (mipp::SupportByteWord ? "yes" : "no") << std::endl;

#ifndef has_max_int8_t
        std::cout << "in this SIMD, int8 max is not inplemented by MIPP" << std::endl;
#endif

#ifndef has_shuff_int8_t
        std::cout << "in this SIMD, int8 shuff is not inplemented by MIPP" << std::endl;
#endif

    std::cout << "----------" << std::endl << std::endl;
}

class Matching{
  private:
    ros::NodeHandle nh;
    ros::Publisher result_pub, edges_pub;
    ros::Subscriber search_num_sub;
    //ros::Subscriber sub;
    image_transport::Subscriber train_sub, test_sub;
    image_transport::ImageTransport it;
    int search_num_;
  public:

    Matching()
      : it(nh)
    {
      result_pub = nh.advertise<geometry_msgs::Polygon>(name_space_ + "matching/result", 1);
      edges_pub = nh.advertise<geometry_msgs::Polygon>(name_space_ + "matching/edges", 1);
      test_sub = it.subscribe(name_space_ + "usb_cam/image_raw", 10, &Matching::testCallback, this);
      train_sub = it.subscribe(name_space_ + "cripped_image", 10, &Matching::trainCallback, this);
      search_num_sub = nh.subscribe(name_space_ + "matching/set_serch_num", 10, &Matching::searchNumCallback, this);
      search_num_ = 1;
    }

    void trainCallback(const sensor_msgs::ImageConstPtr& msg);
    void testCallback(const sensor_msgs::ImageConstPtr& msg);
    void searchNumCallback(const std_msgs::Int32ConstPtr& msg);
};

void Matching::searchNumCallback(const std_msgs::Int32ConstPtr& msg) {
  search_num_ = msg->data;
}

void Matching::testCallback(const sensor_msgs::ImageConstPtr& msg) {
  cv::Mat gray, points_img;
  cv::Mat image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
  cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

  std::vector<cv::Point> point_list;
  std::vector<double> edges_x, edges_y;
  double result_x, result_y, result_angle;
  geometry_msgs::Polygon result_msg;
  geometry_msgs::Point32 point;
  geometry_msgs::Polygon edges_msg;

  bool loop = true;

  while(loop)
  {

    auto ret = angle_test(true, gray, result_x, result_y, result_angle, edges_x, edges_y); // test or train

    loop = ret;

    if ((gray.cols > 0) && (gray.rows > 0))
    {
      points_img = cv::Mat::zeros(gray.rows, gray.cols, CV_8UC1);

      if(ret)
      {
        edges_msg.points.clear();

        point.x = result_x;
        point.y = result_y;
        point.z = result_angle;


        point_list.clear();
        for (int i=0;i<edges_x.size();i++)
        {
          geometry_msgs::Point32 p;
          p.x =(float)edges_x[i];
          p.y = (float)edges_y[i];
          edges_msg.points.push_back(p);

          cv::Point newPoint;
          cv::Point rp;
          rp = rotate(cv::Point((int)p.x,(int)p.y), -result_angle);
          newPoint.x = (int)rp.x + (int)point.x;
          newPoint.y = (int)rp.y + (int)point.y;
          point_list.push_back(newPoint);

          if ((newPoint.x >= 0)&&(newPoint.x <points_img.cols)){
            if ((newPoint.y >= 0)&&(newPoint.y <points_img.rows)){
              points_img.at<uchar>(newPoint.y, newPoint.x) = 255;
            }
          }

    /*
          try{
          }
          catch(...)
          {
          }
          */
        }

        std::vector<cv::Point> approx;
        cv::convexHull(point_list, approx);

        double per = polygon_occupancy(approx, gray);
        std::cout << "per:" << per << std::endl;

        if (per > 0.8)
        {
          result_msg.points.push_back(point);
        }
        else
        {
          loop = 0;
          break;
        }
	if (result_msg.points.size() == search_num_)break;

        cv::fillConvexPoly(points_img, approx, cv::Scalar(255));
        cv::dilate(points_img, points_img, cv::Mat::ones(7, 7, CV_8U ));

        point_list.clear();
        for (int i=0;i<points_img.cols;i++){
          for (int j=0;j<points_img.rows;j++){
            if (points_img.at<uchar>(j,i) == 255)
            {
              cv::Point p;
              p.x = i;
              p.y = j;
              point_list.push_back(p);
            }
          }
        }
        approx.clear();
        cv::convexHull(point_list, approx);

        cv::fillConvexPoly(gray, approx,cv::Scalar(0));
        //cv::imshow("points_image",points_img);
        //cv::imshow("image",gray);
        // closing
        /*
        int morph_size = 2;
        Mat element = getStructuringElement(
        MORPH_RECT,
        Size(2 * morph_size + 1,
              2 * morph_size + 1),
                 Point(morph_size, morph_size));
        cv::morphologyEx(gray, gray,
        cv::MORPH_OPEN, element,
            Point(-1, -1), 2);
        */
      }
      else
      {
        break;
      }
    }
  }

  edges_pub.publish(edges_msg);
  result_pub.publish(result_msg);

  if (DEBUG)
  {
    if ((gray.cols > 0) && (gray.rows > 0))
    {
        //cv::imshow("image",gray);
    }

    if ((points_img.cols > 0) && (points_img.rows > 0))
    {
      //cv::imshow("points_image", points_img);
    }

    cv::waitKey(100);
  }

}

void Matching::trainCallback(const sensor_msgs::ImageConstPtr& msg) {
  cv::Mat gray;
  cv::Mat image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
  cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  angle_train(true, gray);
}

int main(int argc, char *argv[]){

    std::cout << argc << std::endl;
    
    if (argc > 1)
    {
	name_space_ = std::string(argv[1]); 
    	ros::init(argc, argv, name_space_ + "_" + "shape_based_matching");	
    	name_space_ += "/";
    }
    else
    {
    	ros::init(argc, argv, name_space_ + "shape_based_matching");	
    }

    //ros::NodeHandle nh = ros::NodeHandle();
    Matching mc;
    // scale_test("test");
    //angle_train(true); // test or train
    //cv::Mat test_img = cv::imread(prefix+"case1/test.png", CV_LOAD_IMAGE_GRAYSCALE);
    // angle_test(true, test_img); // test or train
    // noise_test("test");
    ros::spin();
    return 0;
}
