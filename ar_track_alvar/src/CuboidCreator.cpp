#include "ar_track_alvar/MultiMarker.h"
#include "ar_track_alvar/Color.h"
#include "highgui.h"
#include "ar_track_alvar/Draw.h"

using namespace std;
using namespace alvar;
//   y <--------
//             |
//             |
//             |x
//             V

struct State {
    IplImage *img;
    stringstream filename;
    double minx, miny, maxx, maxy; // top-left and bottom-right in pixel units
    MultiMarker multi_marker;

    // General options
    bool   prompt;
    double units;           // how many pixels per one unit
    double marker_side_len; // marker side len in current units
    double cuboid_side_len; // cuboid side len in current units
    double cuboid_side_wid; // cuboid side width in current units
    double cuboid_side_hgt; // cuboid side height in current units
    int    marker_type;     // 0:MarkerData, 1:ArToolkit
    int    marker_id;
    double posx, posy;      // The position of marker center in the given units
    double transx, transy, transz;
    double side_len, side_hgt; // The size of the current ROI
    int    rot[9];
    double    content_res;
    double margin_res;
    bool   array;
    unsigned char color;

    // MarkerData specific options
    MarkerData::MarkerContentType marker_data_content_type;

    State()
        : img(0),
          prompt(false),
          units(96.0/2.54),      // cm assuming 96 dpi
          marker_side_len(9.0),  // 9 cm
          cuboid_side_len(9.0),   //9cm
          cuboid_side_wid(9.0),   //9cm
          cuboid_side_hgt(9.0),   //9cm
          marker_type(0),
          marker_id(0),
          side_len(0),side_hgt(0),
          posx(0), posy(0),
          transx(0), transy(0), transz(0),
          content_res(0),        // 0 uses default
          margin_res(0.0),       // 0.0 uses default (can be n*0.5)
          array(false),
          color(0),
          marker_data_content_type(MarkerData::MARKER_CONTENT_TYPE_NUMBER)
    {}
    ~State() {
        if (img) cvReleaseImage(&img);
    }

    IplImage *rotateImage2(const IplImage *src, float angleDegrees)
    {
        // Create a map_matrix, where the left 2x2 matrix
        // is the transform and the right 2x1 is the dimensions.
        float m[6];
        CvMat M = cvMat(2, 3, CV_32F, m);
        int w = src->width;
        int h = src->height;
        float angleRadians = angleDegrees * ((float)CV_PI / 180.0f);
        m[0] = (float)(cos(angleRadians));
        m[1] = (float)(sin(angleRadians));
        m[3] = -m[1];
        m[4] = m[0];
        m[2] = w*0.5f;
        m[5] = h*0.5f;

        // Make a spare image for the result
        CvSize sizeRotated;
        sizeRotated.width = cvRound(h);
        sizeRotated.height = cvRound(w);

        // Rotate
        IplImage *imageRotated = cvCreateImage(sizeRotated,
            src->depth, src->nChannels);

        // Transform the image
        cvGetQuadrangleSubPix(src, imageRotated, &M);

        return imageRotated;
    }

    void AddMarker(const char *id) {
        std::cout<<"ADDING MARKER "<<id<<std::endl;
        MarkerData md(marker_side_len, content_res, margin_res);
        minx = 0;
        miny = 0;
        maxx =(2.*cuboid_side_hgt+2.*cuboid_side_wid)*units+0.5;
        maxy = (cuboid_side_len+2.*cuboid_side_wid)*units+0.5;
        if (img == 0)
        {
            // img = cvCreateImage(cvSize(int(side_len*units+0.5),
            //                           int(side_hgt*units+.5)), IPL_DEPTH_8U, 1);


            img = cvCreateImage(cvSize(int(maxx),
                                    int(maxy)), IPL_DEPTH_8U, 1);

            cvSet(img, cvScalar(255));
            filename.str("");
            filename<<"MarkerData";
                }


          CvRect roi = cvRect(0,0,1,1);
          //
          // cvSetImageROI(img, roi);
          // cvCopy(img, new_img);
          // cvReleaseImage(&img);
          // img = new_img;

          roi.x = int((posx*units) - (side_len*units/2.0) - minx -0.5);
          roi.y = int((posy*units) - (side_hgt*units/2.0) - miny -0.5);
          roi.width = int(side_len*units+0.5); roi.height = int(side_hgt*units+0.5);
          cvSetImageROI(img, roi);
          CvPoint tr = cvPoint(0,0);
          CvPoint tl = cvPoint(0,roi.height);
          CvPoint bl = cvPoint(roi.width,roi.height);
          CvPoint br = cvPoint(roi.width,0);

          cvLine(img, bl , br , color,8, CV_AA, 0);
          cvLine(img, br , tr , color,8, CV_AA, 0);
          cvLine(img, tr , tl , color,8, CV_AA, 0);
          cvLine(img, tl , bl , color,8, CV_AA, 0);


          roi.x = int((posx*units) - (marker_side_len*units/2.0) - minx +0.5);
          roi.y = int((posy*units) - (marker_side_len*units/2.0) - miny +0.5);
          roi.width = int(marker_side_len*units+0.5); roi.height = int(marker_side_len*units+0.5);
          cvSetImageROI(img, roi);


        int idi = atoi(id);
        md.SetContent(marker_data_content_type, idi, 0);
        if (filename.str().length()<64) filename<<"_"<<idi;

        Pose pose;
        pose.Reset();
        pose.SetTranslation(0, 0, 0);
        multi_marker.PointCloudAdd(idi, marker_side_len, pose);
        multi_marker.PointCloudRotate(idi, rot);
        multi_marker.PointCloudTranslate(idi, transx, transy, transz);

        md.ScaleMarkerToImage(img);


        cvResetImageROI(img);
    }


    void Addcuboid()
    {
        if (img)
        {
            cvReleaseImage(&img);
            img = 0;
        }

        for(unsigned int face = 0; face < 6; face++)
        {
          std::cout<<"  marker id ["<< marker_id <<"]: "; std::flush(std::cout);

          if(face == 0)
          {
            assign_rot(0,0,0, 0,0,0, 0,0,0);
            transx = 0; transy = 0; transz = 0;
            posx =  cuboid_side_hgt /2.0;
            posy =(cuboid_side_len )/2.0 + cuboid_side_wid;
            side_hgt = cuboid_side_len;
            side_len = cuboid_side_hgt;
          }
          else if(face == 1)
          {
            assign_rot(0,0,-1, 0,1,0, 1,0,0);
            transx = cuboid_side_len/2.; transy = 0; transz = -cuboid_side_hgt/2.;
            posx = (cuboid_side_wid )/2.0 + cuboid_side_hgt ;
            posy = (cuboid_side_len )/2.0 + cuboid_side_wid;
            side_hgt = cuboid_side_len;
            side_len = cuboid_side_wid;




          }
          else if(face == 2)
          {
            assign_rot(-1,0,0, 0,1,0, 0,0,-1);
            transx = 0; transy = 0; transz = -cuboid_side_hgt;
            posx = (cuboid_side_hgt + cuboid_side_wid ) + cuboid_side_hgt /2.0;
            posy = (cuboid_side_len )/2.0 + cuboid_side_wid;
            side_hgt = cuboid_side_len;
            side_len = cuboid_side_hgt;


          }
          else if(face == 3)
          {
            assign_rot(0,0,1, 0,1,0, -1,0,0);
            transx = -cuboid_side_len/2.; transy = 0; transz = -cuboid_side_hgt/2.;
            posx = 3.*(cuboid_side_hgt + cuboid_side_wid )/2.0+ cuboid_side_hgt /2.0;
            posy = (cuboid_side_len )/2.0 + cuboid_side_wid;
            side_hgt = cuboid_side_len;
            side_len = cuboid_side_wid;
          }
          else if(face == 4)
          {
            assign_rot(1,0,0, 0,0,-1, 0,1,0);
            transx = 0; transy = cuboid_side_wid/2.; transz = -cuboid_side_hgt/2.;
            posx =  cuboid_side_hgt /2.0;
            posy = cuboid_side_wid/2.;
            side_hgt = cuboid_side_wid;
            side_len = cuboid_side_hgt;
          }
          else if(face == 5)
          {
            assign_rot(1,0,0, 0,0,1, 0,-1,0);
            transx = 0; transy = -cuboid_side_wid/2.; transz = -cuboid_side_hgt/2.;
            posx =  cuboid_side_hgt /2.0;
            posy = (cuboid_side_len + 3*cuboid_side_wid/2. );
            side_hgt = cuboid_side_wid;
            side_len = cuboid_side_hgt;


          }

          std::stringstream ss;
          ss<<marker_id;

          AddMarker(ss.str().c_str());
          marker_id++;

        }


    }

    void Save()
    {
        if (img)
        {
          IplImage* color_img = cvCreateImage(cvGetSize(img),IPL_DEPTH_8U,3);
          cvCvtColor(img, color_img, CV_GRAY2RGB);
          col::change_color(color_img, color);
          img = color_img;
          img = rotateImage2(img, 90);

          std::stringstream filenamexml;
          filenamexml<<filename.str()<<".xml";
          filename<<".png";
          std::cout<<"Saving: "<<filename.str()<<std::endl;
          cvSaveImage(filename.str().c_str(), img);

          std::cout<<"Saving: "<<filenamexml.str()<<std::endl;
          multi_marker.Save(filenamexml.str().c_str(), alvar::FILE_FORMAT_XML);
        }
    }

    void assign_rot(int xx, int xy, int xz, int yx, int yy, int yz, int zx, int zy, int zz)
    {
      rot[0] = xx;
      rot[1] = xy;
      rot[2] = xz;
      rot[3] = yx;
      rot[4] = yy;
      rot[5] = yz;
      rot[6] = zx;
      rot[7] = zy;
      rot[8] = zz;
    }
};

int main(int argc, char *argv[])
{
    State st;
    try {
        if (argc < 2) st.prompt = true;
        for (int i=1; i<argc; i++)
        {
            if (strcmp(argv[i],"-u") == 0)
                st.units = atof(argv[++i]);
            else if (strcmp(argv[i],"-uin") == 0)
                st.units = (96.0);
            else if (strcmp(argv[i],"-ucm") == 0)
                st.units = (96.0/2.54);
            else if (strcmp(argv[i],"-sm") == 0)
                st.marker_side_len = atof(argv[++i]);
            else if (strcmp(argv[i],"-sc") == 0) {
                st.cuboid_side_len = atof(argv[++i]);
                st.cuboid_side_hgt = atof(argv[++i]);
                st.cuboid_side_wid = atof(argv[++i]);}
            else if (strcmp(argv[i],"-r") == 0)
                st.content_res = atoi(argv[++i]);
            else if (strcmp(argv[i],"-m") == 0)
                {st.margin_res = atof(argv[++i]); std::cout << st.margin_res << std::endl;}
            else if (strcmp(argv[i],"-p") == 0)
                st.prompt = true;
            else if (strcmp(argv[i],"-c") == 0)
            {
                col::color_t tmp_color = col::get_color(argv[++i][0]);
                st.color = col::get_color(tmp_color);
            }
            else
            {
                std::string s = argv[i];
                if (s.length() > 0) st.marker_id=atoi(s.c_str());
                st.Addcuboid();

                st.Save();

            }
        }

        // Output usage message
        if (st.prompt)
        {
            std::string filename(argv[0]);
            filename = filename.substr(filename.find_last_of('\\') + 1);
            std::cout << "#=============#" << std::endl;
            std::cout << "= cuboidCreator =" << std::endl;
            std::cout << "#=============#" << std::endl;
            std::cout << std::endl;
            std::cout << "Description:" << std::endl;
            std::cout << "  This is a utility to generate pattern with markers for to create cuboids." << std::endl;
            std::cout << "  This will generate both the PNG and XML files where you run this utility." << std::endl;
            std::cout << "  Once the PNG is printed, do not cut all markers independently, but only the entire pattern." << std::endl;
            std::cout << std::endl;
            std::cout << "Options usage:" << std::endl;
            std::cout << "  " << filename << " [options] argument" << std::endl;
            std::cout << std::endl;
            std::cout << "    65535             marker with number 65535" << std::endl;
            std::cout << "    -u 96             use units corresponding to 1.0 unit per 96 pixels" << std::endl;
            std::cout << "    -uin              use inches as units (assuming 96 dpi)" << std::endl;
            std::cout << "    -ucm              use cm's as units (assuming 96 dpi) <default>" << std::endl;
            std::cout << "    -sm 5.0           use marker size 5.0x5.0 units (default 9.0x9.0)" << std::endl;
            std::cout << "    -sc 5.0  4.0 3.0  use cuboid size 5.0x4.0x3.0 units (default 9.0x9.0x9.0)" << std::endl;
            std::cout << "    -r 5              marker content resolution -- 0 uses default" << std::endl;
            std::cout << "    -m 1.0            marker margin resolution -- 0 uses default" << std::endl;
            std::cout << "    -p                prompt marker placements interactively from the user" << std::endl;
            std::cout << "    -c B              cuboid color :  B:blue R:red G:green P:pink S:sky Y:yellow -- default black" << std::endl;
            std::cout << std::endl;

            // Interactive stuff here
            st.marker_type = 0;
            st.marker_data_content_type = MarkerData::MARKER_CONTENT_TYPE_NUMBER;
            std::cout<<"\nPrompt marker placements interactively"<<std::endl;
            std::cout<<"  units: "<<st.units/96.0*2.54<<" cm "<<st.units/96.0<<" inches"<<std::endl;
            std::cout<<"  marker side: "<<st.marker_side_len<<" units"<<std::endl;
            std::cout << std::endl;
            std::cout << std::endl;

            bool loop=true;
            std::string s;

            while(loop)
            {
                std::cout<<"  marker id (use -1 to end and ENTER to automaticaly increase) ["<<st.marker_id<<"]: "; std::flush(std::cout);
                std::getline(std::cin, s); if (s.length() > 0) st.marker_id=atoi(s.c_str());
                if (st.marker_id < 0) break;

                std::cout<<"  Marker size (cm): "; std::flush(std::cout);
                std::getline(std::cin, s); if (s.length() > 0) st.marker_side_len = atof(s.c_str());

                st.cuboid_side_len = st.marker_side_len;
                std::cout<<"  cuboid length (cm): "; std::flush(std::cout);
                std::getline(std::cin, s);
                if (s.length() > 0)
                    st.cuboid_side_len = atof(s.c_str());



                st.cuboid_side_hgt = st.marker_side_len;
                std::cout<<"  cuboid height (cm): "; std::flush(std::cout);
                std::getline(std::cin, s);
                if (s.length() > 0)
                    st.cuboid_side_hgt = atof(s.c_str());

                st.cuboid_side_wid = st.marker_side_len;
                std::cout<<"  cuboid width (cm): "; std::flush(std::cout);
                std::getline(std::cin, s);
                if (s.length() > 0)
                    st.cuboid_side_wid = atof(s.c_str());


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

                st.Addcuboid();
                st.Save();

                st.multi_marker = MultiMarker();
            }
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
