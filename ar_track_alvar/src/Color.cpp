#include "ar_track_alvar/Color.h"

namespace col
{
  color_t get_color(char color_name)
  {
    color_t color = COLOR_BLACK;
    switch(color_name)
    {
      case 'R': case 'r': color = COLOR_RED; break;
      case 'B': case 'b': color = COLOR_BLUE; break;
      case 'G': case 'g': color = COLOR_GREEN; break;
      case 'Y': case 'y': color = COLOR_YELLOW; break;
      case 'P': case 'p': color = COLOR_PINK; break;
      case 'S': case 's': color = COLOR_SKY; break;
      default: color = COLOR_BLACK; break;
    }
    return color;
  }

  unsigned char get_color(color_t p_color)
  {
    unsigned char result = 0;
    if((p_color == COLOR_RED) || (p_color == COLOR_YELLOW) || (p_color == COLOR_PINK))
      result |= 0x04;
    if((p_color == COLOR_GREEN) || (p_color == COLOR_YELLOW) || (p_color == COLOR_SKY))
      result |= 0x02;
    if((p_color == COLOR_BLUE) || (p_color == COLOR_PINK) || (p_color == COLOR_SKY))
      result |= 0x01;
    return result;
  }

  void change_color(cv::Mat& color_img, unsigned char p_color)
  {
    cv::Mat mask;
    cv::inRange(color_img, cv::Scalar(0, 0, 0), cv::Scalar(0, 0, 0), mask);
    color_img.setTo(cv::Scalar(((p_color >> 0)&0x01)*200, ((p_color >> 1)&0x01)*150, ((p_color >> 2)&0x01)*150), mask);
  }
}
