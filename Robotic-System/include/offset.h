#ifndef OFFSET_H
#define OFFSET_H

#include <fstream>
#include <math.h>
#include <vector>
#include <iostream>
#include <algorithm>

using namespace std;

bool cmp2(pair<double,pair<double,double>> a, pair<double,pair<double,double>> b)
{
    return a.first < b.first;
}

namespace IRIM
{
   
class OffSolver
{
public:
    OffSolver()
    {
        y_list.resize(0);
        x_offset_list.resize(0);  
        y_offset_list.resize(0);        
        save_sort_list.resize(0);
    }
    bool read_from_file(std::string sFileName)
    {   
        save_sort_list.resize(0);
        std::ifstream inFile;
        inFile.open(sFileName.c_str(), std::ios::in);
        if(inFile.fail())
        {
            fprintf(stderr, "file %s open error\n", sFileName.c_str());
            return false;
        }
        std::vector<std::string> string_buffer;
        std::string pattern = ";"; 
        //开始做数字读取
        do
        {
            std::string sLine;
            std::getline(inFile, sLine);
            string_buffer.push_back(sLine);
        }while (!inFile.eof());

        for(int i = 2; i <string_buffer.size()-1; i++)
        {
            std::vector<std::string> sub_string = string_split(string_buffer.at(i), pattern); 
            double y = std::atof(sub_string.at(0).c_str());
            double x_offset = std::atof(sub_string.at(2).c_str());
            double y_offset = std::atof(sub_string.at(1).c_str());
            x_offset_list.push_back(x_offset);
            y_offset_list.push_back(y_offset);
            pair<double,double> Offset_list;
            Offset_list.first = x_offset;
            Offset_list.second = y_offset;
            pair<double,pair<double,double>> all;
            all.first = y;
            all.second = Offset_list;
            save_sort_list.push_back(all);
            

        }
        return true;
        
    };

    pair<double,double> give_the_offset(double current_y)
    {
        vector<pair<double,pair<double,double>>> sort_list = save_sort_list;
        for(int i = 0; i < sort_list.size(); i ++)
        {
            double temp = fabs(sort_list.at(i).first - current_y);
            sort_list.at(i).first = temp;
        }
        //升序排列
        std::sort(sort_list.begin(), sort_list.end(), cmp2);
        for(int i = 0; i < sort_list.size(); i ++)
        {
            cout << sort_list.at(i).first << "->";
        }
        cout << endl;
        pair<double,double> result;
        //offset
        double x1, x2;
        double y1, y2;
        x1 = sort_list.at(0).second.first;
        x2 = sort_list.at(1).second.first;
        y1 = sort_list.at(0).second.second;
        y2 = sort_list.at(1).second.second;
        cout << "x1 =" << x1 << endl;
        cout << "x2 =" << x2 << endl;
        cout << "y1 =" << y1 <<endl;
        cout << "y2 =" << y2 <<endl;
        
        double The_both = sort_list.at(0).first + sort_list.at(1).first;

        if(The_both == 0)
        {
            The_both = 0.000001;
        }
        else
        {
            result.first = (sort_list.at(0).first / The_both) * x2 + (sort_list.at(1).first / The_both) * x1;
        }
        if(The_both == 0)
        {
            The_both = 0.000001;
        }
        else
        {
            result.second = (sort_list.at(0).first / The_both) * y2 + (sort_list.at(1).first / The_both) * y1;
        }
        std::cout <<"For y " << current_y << endl;
        std::cout <<" The x_offset result is " << result.first << std::endl;
        std::cout <<" The y_offset result is " << result.second << std::endl;

        
        return result;
    };

    vector<double> y_list; 
    
    vector<double> x_offset_list; 
    vector<double> y_offset_list; 
    // y_distance / <x_offset, y_offset>
    vector<pair<double,pair<double,double>>> save_sort_list;
    std::vector<std::string> string_split(std::string str, std::string pattern)
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
    };

};
};

#endif /* OFFSET_H */
