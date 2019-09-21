#include <iostream>
#include <fstream>

namespace IRIM
{
    class DataInput
    {
    public:
        DataInput()
        {
            fs.open("/home/logisticrobot/Desktop/IRIM_EVENTUAL/src/conmute_pythonCpp/label_result.txt");
        }
        ~DataInput(){
            fs.close();
        }
        void writeData(int execute_time)
        {
            switch(execute_time)
            {
                
                case 0:
                inner_writer("biscuit");
                break;
                case 1:
                inner_writer("cola");
                break;
                case 2:
                inner_writer("tissue");
                break;
                case 3:
                inner_writer("chips");
                break;
                case 4:
                inner_writer("milk");
                break;
                case 5:
                inner_writer("toothpaste");
                break;
            }

        }
    private:
        std::ofstream fs; 
        void inner_writer(std::string Input)
        {
            fs << "['label', 'pro', 'cen_x', 'cen_y', 'w', 'h']" << std::endl;
            fs << "['" << Input << "', 0.9649609923362732, 174.01422119140625, 373.92138671875, 70.23097229003906, 118.57609558105469]" << std::endl;
        }

    };
}