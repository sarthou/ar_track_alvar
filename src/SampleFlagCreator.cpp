#include "ar_track_alvar/MultiMarker.h"
#include "ar_track_alvar/color.h"
#include "highgui.h"
using namespace std;
using namespace alvar;

struct State {
    IplImage *img;
    stringstream filename;
    double minx, miny, maxx, maxy; // top-left and bottom-right in pixel units
    MultiMarker multi_marker;

    // General options
    bool   prompt;
    double units;           // how many pixels per one unit
    double marker_side_len; // marker side len in current units
    int    marker_type;     // 0:MarkerData, 1:ArToolkit
    double posx, posy, posz;      // The position of marker center in the given units
    int    content_res;
    double margin_res;
    bool   array;
    unsigned char color;

    // MarkerData specific options
    MarkerData::MarkerContentType marker_data_content_type;
    bool                          marker_data_force_strong_hamming;

    State()
        : img(0),
          prompt(false),
          units(96.0/2.54),      // cm assuming 96 dpi
          marker_side_len(9.0),  // 9 cm
          marker_type(0),
          posx(0), posy(0),
          content_res(0),        // 0 uses default
          margin_res(0.0),       // 0.0 uses default (can be n*0.5)
          marker_data_content_type(MarkerData::MARKER_CONTENT_TYPE_NUMBER),
          marker_data_force_strong_hamming(false),
          color(0)
    {}
    ~State() {
        if (img) cvReleaseImage(&img);
    }

    void AddMarker(const char *id) {
        std::cout<<"ADDING MARKER "<<id<<std::endl;

        MarkerData md(marker_side_len, content_res, margin_res);
        int side_len = int(marker_side_len*units+0.5);
        if (img == 0) {
            img = cvCreateImage(cvSize(side_len, side_len), IPL_DEPTH_8U, 1);
            filename.str("");
            filename<<"MarkerData";
        }

        int idi = atoi(id);
        md.SetContent(marker_data_content_type, idi, 0);
        if (filename.str().length()<64) filename<<"_"<<idi;

        Pose pose;
        pose.Reset();
        pose.SetTranslation(posx, -posy, posz);
        multi_marker.PointCloudAdd(idi, marker_side_len, pose);

        md.ScaleMarkerToImage(img);
        cvResetImageROI(img);
        IplImage* color_img = cvCreateImage(cvGetSize(img),IPL_DEPTH_8U,3);
        cvCvtColor(img, color_img, CV_GRAY2RGB);
        col::change_color(color_img, color);
        img = color_img;
    }

    void Save() {
        if (img) {
            std::stringstream filenamexml;
            filenamexml<<filename.str()<<".xml";
            filename<<".png";
            std::cout<<"Saving: "<<filename.str()<<std::endl;
            cvSaveImage(filename.str().c_str(), img);

            std::cout<<"Saving: "<<filenamexml.str()<<std::endl;
            multi_marker.Save(filenamexml.str().c_str(), alvar::FILE_FORMAT_XML);

        }
    }
};

void print_tag()
{
  std::cout << std::endl;
  std::cout << "positive height" << std::endl;
  std::cout << "   ^   " << std::endl;
  std::cout << "   |   " << std::endl;
  std::cout << "#######" << std::endl;
  std::cout << "#  #  #" << std::endl;
  std::cout << "#  #  #" << std::endl;
  std::cout << "#     # --> positive side" << std::endl;
  std::cout << "#     #" << std::endl;
  std::cout << "#     #" << std::endl;
  std::cout << "#######" << std::endl;
  std::cout << std::endl;
}

int main(int argc, char *argv[])
{
    try {
        // Output usage message
        std::string filename(argv[0]);
        filename = filename.substr(filename.find_last_of('\\') + 1);
        std::cout << "SampleFlagCreator" << std::endl;
        std::cout << "===================" << std::endl;
        std::cout << std::endl;
        std::cout << "Description:" << std::endl;
        std::cout << "  This is an example of how to use the 'MarkerData'" << std::endl;
        std::cout << "  classes to generate marker images. This application can be used to" << std::endl;
        std::cout << "  generate flag markers setups that can be used with" << std::endl;
        std::cout << "  SampleMarkerDetector and SampleMultiMarker." << std::endl;
        std::cout << std::endl;

        // Interactive stuff here
        State ref_st;
        ref_st.marker_type = 0;
        ref_st.marker_data_content_type = MarkerData::MARKER_CONTENT_TYPE_NUMBER;
        std::cout<<"\nPrompt marker placements interactively"<<std::endl;
        std::cout<<"  units: "<<ref_st.units/96.0*2.54<<" cm "<<ref_st.units/96.0<<" inches"<<std::endl;
        std::cout << std::endl;
        std::cout << std::endl;

        bool loop=true;
        std::string s;
        int marker_id=0;
        double posx=0.0, posy=0.0, posz=0.0;
        bool vert=false;

        unsigned last_id = 0;
        std::cout<<"  Last marker id you use: "; std::flush(std::cout);
        std::getline(std::cin, s); if (s.length() > 0) last_id=atoi(s.c_str());

        while(loop)
        {
            State st;
            std::stringstream ss;

            std::cout<<"  marker id ["<< ++last_id <<"]: "; std::flush(std::cout);
            marker_id = last_id;

            std::cout<<"  Marker size (cm): "; std::flush(std::cout);
            std::getline(std::cin, s); if (s.length() > 0) st.marker_side_len = atof(s.c_str());

            print_tag();
            std::cout<<"  Flag side translation : "; std::flush(std::cout);
            std::getline(std::cin, s); if (s.length() > 0) posx=atof(s.c_str());
            std::cout<<"  Flag height translation : "; std::flush(std::cout);
            std::getline(std::cin, s); if (s.length() > 0) posy=atof(s.c_str());
            std::cout<<"  Flag depth translation : "; std::flush(std::cout);
            std::getline(std::cin, s); if (s.length() > 0) posz=atof(s.c_str());
            st.posx=-posx;
            st.posy=posy;
            st.posz=-posz;
            std::cout<<"  Colors :" << std::endl;
            std::cout<<"  B - blue" << std::endl;
            std::cout<<"  R - red" << std::endl;
            std::cout<<"  G - green" << std::endl;
            std::cout<<"  P - pink" << std::endl;
            std::cout<<"  S - sky" << std::endl;
            std::cout<<"  Y - yellow" << std::endl;
            std::cout<<"  default - black :" << std::endl;
            std::cout<<"  Marker color : "; std::flush(std::cout);
            std::getline(std::cin, s); if (s.length() <= 0) s = " ";
            col::color_t tmp_color = col::get_color(s[0]);
            st.color = col::get_color(tmp_color);

            ss<<marker_id;
            st.AddMarker(ss.str().c_str());

            st.Save();

            bool ok = false;
            bool other = false;
            while(!ok)
            {
              std::cout<<"  Create an other flag [y/n]: "; std::flush(std::cout);
              std::getline(std::cin, s);
              if(s == "y" || s == "Y")
              {
                ok = true;
                other = true;
                std::cout << std::endl << "----------------" << std::endl;
              }
              else if(s == "n" || s == "N")
                ok = true;
            }

            if(!other)
              break;
        }

        return 0;
    }
    catch (const std::exception &e) {
        std::cout << "Exception: " << e.what() << endl;
    }
    catch (...) {
        std::cout << "Exception: unknown" << std::endl;
    }
}
