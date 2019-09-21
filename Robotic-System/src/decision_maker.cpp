
#include "decision_maker.h"


 namespace IRIM
{   
    bool DeepLearningDM::run()
    {   
        save_a_picture();
        picture_recongnize();
        show_the_result();
        if(help())
        {
            ros::NodeHandle n;
            static ros::Publisher pub_2D = n.advertise<geometry_msgs::Pose2D>("/IRIM/pub_2D",1);
            sleep(1);
            pub_2D.publish(pose2d);
            ros::spinOnce();
            return true;
        };
        return false;

    }

    void DecisionMaker::save_a_picture()
    {   
        std::cout << "save a picture" << std::endl;
        ptrNodeHandle = NodeHandlePtr(new ros::NodeHandle());
        static ros::Subscriber sub = ptrNodeHandle->subscribe("/kinect2/qhd/image_color_rect", 15, save_pic_callBack);
        ros::spinOnce();        
        sleep(2);
        ros::spinOnce();
    }

    RecoResult& DecisionMaker::get_result()
    {
        assert(!out_Rect.empty());

        return out_Rect;
    }

    std::vector<bool> DecisionMaker::get_plan_result()
    {
        if(!desion_result.empty())
        {
            return desion_result;
        };
    }

    std::vector<std::string> DecisionMaker::string_split(std::string str, std::string pattern)
    {
        std::vector<std::string> string_ret;
        if(pattern.empty()) return string_ret;
        size_t start = 0;
        size_t index = str.find_first_of(pattern, 0);
        while(index!=str.npos)
        {
            if(start !=index) string_ret.push_back(str.substr(start, index-start));
            start = index +1;
            index = str.find_first_of(pattern, start);            

        }
        if(!str.substr(start).empty()) string_ret.push_back(str.substr(start));
        return string_ret;
    }


    void DecisionMaker::read_from_file(std::string sFileName)
    {  
        std::ifstream inFile;
        inFile.open(sFileName.c_str(), std::ios::in);
        if(inFile.fail())
        {
            fprintf(stderr, "file %s open error\n", sFileName.c_str());
        }
        std::vector<std::string> string_buffer;
        std::string pattern = ", ";
        do
        {
            std::string sLine;
            std::getline(inFile, sLine);
            string_buffer.push_back(sLine);

        }while (!inFile.eof());

        for(int i = 1; i <string_buffer.size()-1;i++)
        {
            out_Rect.resize(1);
            std::vector<std::string> sub_string;
            std::string temp = ""; 
            temp = string_buffer[i].substr(1, string_buffer[i].size()-2);
            sub_string = string_split(temp, pattern);
            cv::Rect2f temp_rect(std::atof(sub_string[2].c_str()), std::atof(sub_string[3].c_str()), 
            std::atof(sub_string[4].c_str()), std::atof(sub_string[5].c_str()));
            std::pair<cv::Rect2f, std::string> temp_pair(temp_rect, sub_string[0].substr(1, sub_string[0].size()-2));
            std::cout <<  "name = " << temp_pair.second << std::endl;
            std::cout <<"(" <<  temp_rect.x << "," << temp_rect.y <<  "," << temp_rect.width <<  "," << temp_rect.height << ")"<< std::endl;
            out_Rect.at(0) = temp_pair;
        }
    }

    bool DecisionMaker::is_vertical_plan(std::string object_type)
    {
        using namespace std;
        cout << "\033[47;30m"<< object_type <<"\033[0m" << std::endl;
        //vertical
        if( 
        !strcmp(object_type.c_str(), "toothpaste")||
        !strcmp(object_type.c_str(), "toothbrush")||
        !strcmp(object_type.c_str(), "book")||
        !strcmp(object_type.c_str(), "gutta pertscha")||
        !strcmp(object_type.c_str(), "oreo")||
        !strcmp(object_type.c_str(), "chips")||
        !strcmp(object_type.c_str(), "biscuit")||
        !strcmp(object_type.c_str(), "french fries")||
        !strcmp(object_type.c_str(), "calcium tablets")|| 
        !strcmp(object_type.c_str(), "orange")||
        !strcmp(object_type.c_str(), "tissue")||
        !strcmp(object_type.c_str(), "cup")||
        !strcmp(object_type.c_str(), "sausage"))
        {
            cout << "\033[47;30m"<< "vertical" <<"\033[0m" << std::endl; 
            return true;
        }
        //horizonal
        else if(            
        !strcmp(object_type.c_str(), "shampoo")||       
        !strcmp(object_type.c_str(), "orange juice")||
        !strcmp(object_type.c_str(), "milk")||
        !strcmp(object_type.c_str(), "spray")||
        !strcmp(object_type.c_str(), "cola"))
        {
            cout << "\033[47;30m"<< "horital" <<"\033[0m" << std::endl; 
            return false;
        }

    }


    bool DecisionMaker::help()
    {
        std::cout << "-----------------help------------------" << std::endl;
        std::cout << "Look at the picture and check the result. " << std::endl;
        std::cout << "The object is " << get_result().at(0).second << " and the picking plan is ";
        if(is_vertical_plan(get_result().at(0).second))
        {

            std::cout << "vertical pick " << std::endl;
        }           
        else
        {
            std::cout << "horizonal pick " << std::endl;
        } 
        std::cout << "Do you want to continue?(press 'y' to continue and 'n' to out)" << std::endl;
        char temp;
        std::cin >> temp;
        std::cin.clear();
        cv::destroyWindow("MyWindow");
        return (temp == 'y')?true:false;

    };
    void DeepLearningDM::picture_recongnize()
    {
        // system("python /home/neousys/Desktop/darknet/darknet.py");
        // system("python /home/logisticrobot/darknet/darknet/darknet.py");
        system("python /home/neousys/Desktop/darknet/darknet.py");
        
    };

    bool DecisionMaker::show_the_result()
    {
        using namespace std;
        std::cout << "show the result" << std::endl;
        cv::Mat saved_pic = cv::imread("/home/neousys/Desktop/IRIM_EVENTUAL/src/conmute_pythonCpp/1.jpg");
        this->read_from_file("/home/neousys/Desktop/IRIM_EVENTUAL/src/conmute_pythonCpp/label_result_false.txt");

        cv::Rect2f temp_rect = this->get_result().at(0).first;
        //cv::Point2f center_point((temp_rect.y + 0.5 * temp_rect.height),(temp_rect.x + 0.5 * temp_rect.width));
        cv::Point2f center_point((temp_rect.x),(temp_rect.y));
        pose2d.x =center_point.x;
        pose2d.y =center_point.y;
        pose2d.theta = 0;
        std::ostringstream objectpoint_str;
        objectpoint_str << this->get_result().at(0).second << ":( " << center_point.x << ", " << center_point.y << " )";
        cv::putText(saved_pic, objectpoint_str.str(), cv::Point(center_point.x, center_point.y+10), cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 0, 0), 1, CV_AA);
        cv::circle(saved_pic, center_point, 2, cv::Scalar(255,0,0), 2);
        cv::namedWindow("MyWindow", CV_WINDOW_AUTOSIZE);
        cv::imshow("MyWindow", saved_pic);
        cv::waitKey(1);
    };

    void DecisionMaker::save_pic_callBack(const sensor_msgs::Image::ConstPtr& msg)
    {   
        cv::Mat saved_pic = cv_bridge::toCvShare(msg, "bgr8")->image;
        
        cv::imwrite("/home/neousys/Desktop/IRIM_EVENTUAL/src/conmute_pythonCpp/1.jpg", saved_pic);
    };

}