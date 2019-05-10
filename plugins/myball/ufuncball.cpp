/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@oersted.dtu.dk                                                    *
 *                                                                         *
 *   xx 03 2007 Modified by Lars                                           *
 *   See svn log for modification history                                  *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Lesser General Public License as        *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Lesser General Public License for more details.                   *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this program; if not, write to the                 *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifdef OPENCV2
#include <legacy/compat.hpp>
#endif

#include <urob4/usmltag.h>
#include <cstdlib>
#include <boost/concept_check.hpp>
#include <ucam4/ufunctioncambase.h>
//#include "../aupoly/urespoly.h"
#include "urespoly.h"
#include <urob4/uresposehist.h>

/**
Example plugin to find balls in camera image
@author Christian Andersen
*/
class UFuncBall : public UFunctionCamBase
{ // NAMING convention recommend that the main plugin function class
  // starts with UFunc followed by
  // a descriptive extension for this specific plugin
public:
  /**
  Constructor */
  UFuncBall()
  {
    setCommand("bf7", "bf7", "camera based ball detect (compiled " __DATE__ " " __TIME__ ")");
    // create global variables
    createBaseVar();
    // initialize local variables
    ellCnt = 0;
  };
  /**
  Destructor - to delete the resource (etc) when finished */
  virtual ~UFuncBall()
  { // possibly remove allocated variables here - if needed
  }
  /**
  Handle incomming command
  Must return true if the function is handled -
  otherwise the client will get a 'failed' reply */
  virtual bool handleCommand(UServerInMsg * msg, void * extra)
  { // handle command(s) send to this plug-in
    const int MRL = 2000;
    char reply[MRL];
    bool ask4help;
//     const int MVL = 50;
//     char val[MVL];
//     int camDevice = -1;
//     bool gotImg = false;
//     int imgPoolNum = -1;
    UImage * img = NULL;
    USmlTag tag;
//     UCamPush * cam = NULL;
    bool result;
    bool debug = true; // default is debug on
    bool smrcl = false; // default is <ball ...> reply
    bool gotBlue = false;
    // check for parameters - 'help' (on line help)
    ask4help = msg->tag.getAttValue("help", NULL, 0);
    if (not ask4help)
    { // get all other parameters
//       msg->tag.getAttValueInt("device", &camDevice);
//       gotImg = msg->tag.getAttValueInt("img", &imgPoolNum);
      msg->tag.getAttValueBool("debug", &debug, true);
      msg->tag.getAttValueBool("smrcl", &smrcl, true);
      msg->tag.getAttValueBool("blue", &gotBlue, true);
    }
    if (ask4help)
    { // create the reply in XML-like (html - like) format
      sendHelpStart("B2");
      sendText("--- available B2 options\n");
      sendText("device=X          Use this camera - for position and parameters\n");
      sendText("img=X             Get image from image pool - else take new image\n");
      sendText("blue              Try find a blue ball (def is red)\n");
      sendText("debug=false       More images and print on server console (def=true)\n");
      sendText("smrcl             Format the reply for MRC (<vision vis1=\"x.x\" vis2=\"y.y\" .../>\n");
      sendText("v2                use version 2 code (edge extraction)");
      sendText("help              This message\n");
      sendHelpDone();
      result = true;
    }
    else
    { // resource is available, so make a reply
      img = getImage(msg, extra);
      result = img != NULL;
      // time to kick some ass
      if (result)
      { // there is an image, make the required ball analysis
        result = findBall(getCam(img->camDevice), img, debug, gotBlue);
        if (smrcl)
        { // format for MRC
          snprintf(reply, MRL, "<vision vis1=\"%d\" vis2=\"%g\" vis3=\"%g\" vis4=\"%d\"/>\n",
                                         result, getPos().x, getPos().y, getCnt());
          sendMsg(msg, reply);
        }
        else if (not result)
        { // did not find any balls in image - with a reasonable size
          snprintf(reply, MRL, "<%s cnt=\"%d\"/>\n", msg->tag.getTagName(), getCnt());
          sendMsg(msg, reply);
        }
        else
        { // send XML open tag with ball count as attribute
          snprintf(reply, MRL, "<%s cnt=\"%d\">\n", msg->tag.getTagName(), getCnt());
          sendMsg(msg, reply);
          // code position of the ball and send
          tag.codePosition(getPos(), reply, MRL, "ball");
          result = sendMsg(msg, reply);
          // send XML close tag
          snprintf(reply, MRL, "</%s>\n", msg->tag.getTagName());
          sendMsg(msg, reply);
        }
      }
      else
        sendWarning("failed to get image");
    }
    // return true if the function is handled with a positive result
    return result;
  }

protected:

  /**
   * Find red ball in image, using parameters from global var pool
   * \param cam camera from wich the image was taken (has focal length and camera pose)
   * \param img is a pointer to the image to be analized
   * \param debug if true, then more images are producen in the image pool
   * \param blue if true, then the blue color values are used (else the red values)
   * \returns true if at least one red ball is found */
  bool findBall(UCamPush * cam, UImage * img, bool debug, bool blue)
  {
    bool found;
    UTime t;
    double ballDiam;
    UImage * mask, *dbgImg = NULL, *maskOrg;
    UImagePool * imgPool;
    UPixel gray(127,127,127);
    // get pointer to image pool plug-in
    imgPool = (UImagePool*)getStaticResource("imgPool", true);
    found = (img != NULL);
    if (found)
    { // an image is available, so ...
      // save source image size for border analysis
      imgSizeW = img->width();
      imgSizeH = img->height();
      // get image for result of colour masking
      mask = imgPool->getImage(varPoolImg->getInt() + 1, true);
      // mask the colours
      maskColors(img, mask, blue);
      // find balls in the
      if (debug)
      { // make a copy of the mask (as the mask image is destroyed by the
        // 'ballCandidate' function
        maskOrg = imgPool->getImage(varPoolImg->getInt() + 2, true);
        maskOrg->copy(mask);
        maskOrg->setName("masked-pixels");
        maskOrg->updated();
        //
        // get debug image for found balls
        dbgImg = imgPool->getImage(varPoolImg->getInt() + 3, true);
        // use original image as background
        dbgImg->copy(img);
        dbgImg->setName("result-circles");
        // but a bit faint (50% towards black)
        dbgImg->tone(gray, 50);
      }
      else
        dbgImg = NULL;
      //
      // filter mask
      filterMask(mask, mask);
      //
      // analyse the masked area and fit to ellipses
      found = findBallCandidates(mask, dbgImg);
    }
    // get time image were captured
    t = img->imgTime;
    if (blue)
    { // set number of found balls
      varBlueCnt->setValued(ellCnt);
      // save image time as detection time for red balls
      varBlueTime->setValued(t.getDecSec());
    }
    else
    { // set number of found balls
      varRedCnt->setValued(ellCnt);
      // save image time as detection time for red balls
      varRedTime->setValued(t.getDecSec());
    }
    // Convert ball position to robot coordinates
    if (found)
    { // get known real size of ball (in meters)
      ballDiam = varBallSize->getValued();
      // find ball position
      found = calculateBallPosition(cam, dbgImg, ballDiam);
      // make ball to display polygon
      makeDisplayPolygon(&ballPos, blue, varBallSize->getDouble(), img->imgTime);
    }
    if (dbgImg != NULL)
      dbgImg->updated();
    if (found)
    { // the ball position is found too, ...
      // save result into global variables
      if (blue)
      { // save blue values
        varBlueCnt->setValued(ellCnt, 0);
        varBluePos->set3D(&ballPos);
      }
      else
      { // save red values
        varRedCnt->setValued(ellCnt, 0);
        varRedPos->set3D(&ballPos);
      }
    }
    return found;
  }

  /**
  Find balls in this image.
  This function is almost a direct
  copy of the 'fitellipse.c' example from the opencv package.
  \param srcImg is a pointer to the image to be analized
  \param dstImg is where the pixels with the ball color is saved.
  \param debug if true, then more images are producen in the image pool
  \param blue are we looking for a blue ball (else a red) */
  void maskColors(UImage * srcImg, UImage * dstImg, bool blue)
  {
    //bool result = false;
    int row, col;
    UPixel * pixYUV;
    unsigned char * pixMask;
    int topLine;
    int uMin, uMax, vMin, vMax;
    UVariable * var;
    //
    UImage * iyuv = imgPool->getImage(varPoolImg->getInt() + 4, true);
    
    // ensure source image is in YUV format
    if (not srcImg->isYUV())
    { // change colour format
      srcImg->toYUV(iyuv);
      iyuv->updated();
    }
    iyuv->setName("source");
    //
    // get cromatisity threashold values (in 0..255 range) from global var settings
    if (blue)
      var = varBlueLim;
    else
      var = varRedLim;
    uMin = var->getInt(0);
    uMax = var->getInt(1);
    vMin = var->getInt(2);
    vMax = var->getInt(3);
    // prepare destination image
    // copy image time and source from original
    dstImg->copyMeta(srcImg, false);
    // set size and colour depth - one plane of 8-bit
    dstImg->setSize(srcImg->getHeight(), srcImg->getWidth(), 1, 8, "gray");
    // and give it a nice name
    dstImg->setName("mask");
    //
    topLine = varTopLine->getInt();
    // apply given colour threashold and put result into image 2
    for (row = 0; row < (int)srcImg->getHeight(); row++)
    { // get pixel pointer (3-plane) to source image start of row
      pixYUV = iyuv->getLine(row);
      // get pointer to destination pixel (same row)
      pixMask = (unsigned char *) dstImg->getLine(row);
      for (col = 0; col < (int)srcImg->getWidth(); col++)
      { // plane 1 (p1) is Y, plane 2 (p2) is U and plane 3 (p3) is V
        if (row > topLine and
            pixYUV->p2 > uMin and pixYUV->p2 < uMax and
            pixYUV->p3 > vMin and pixYUV->p3 < vMax)
          // right colou - set to large value (white)r
          *pixMask = 255;
        else
          // not ball colour - so black
          *pixMask = 0;
        // increase to next destination pixel
        pixMask++;
        // increase to next source pixel
        pixYUV++;
      }
    }
    dstImg->updated();
  }


  /**
  Find balls in this image.
  This function is almost a direct
  copy of the 'fitellipse.c' example from the opencv package.
  \param img is a pointer to a BW mask image with interesting pixels beeing > 0
  \param ellImg optional image where found ellipses are drawn (for debug)
  \returns true if at least one ball candidate is found. */
  bool findBallCandidates(UImage * img, UImage * ellImg)
  {
    //bool result = false;
    CvMemStorage* stor;
    CvSeq* cont;
    CvPoint* PointArray;
    CvPoint2D32f* PointArray2D32f;
    int i, count;
    float eMaj, eMin;
    CvPoint center;
    CvSize size;
    float angle;
    int minSize = 8; // minimum pixel size of found colour blocks
    int maxSize;
    double ratio, ratioLim;
    //
    // Using openCV to find structures in the mask image.
    // OpenCV likes dinamic storage allocation, so here is some space allocated
    stor = cvCreateMemStorage(0);
    // create sequence structure for found structures
    cont = cvCreateSeq(CV_SEQ_ELTYPE_POINT, sizeof(CvSeq), sizeof(CvPoint) , stor);
    // Find all contours.in image 4
    cvFindContours( img->cvArr(), stor, &cont, sizeof(CvContour),
                    /*CV_RETR_LIST*/ CV_RETR_EXTERNAL
                        , CV_CHAIN_APPROX_NONE, cvPoint(0,0));
    //
    ratioLim = varMajMinRatio->getDouble();
    maxSize = varMaxSize->getInt();
    // Now the found contours will be estiamted as ellipses
    // if they larger than 4x4 pixels
    ellCnt = 0;
    //
    // This cycle draw all contours and approximate it by ellipses.
    // continue as long as there is contoures in the contour sequence
    for(;cont != NULL;cont = cont->h_next)
    { // find interesting shapes
      count = cont->total; // This is number point in contour
      //
      // Number point must be more than or equal to 6 (for cvFitEllipse_32f).
      if (count < 10)
        // too small for ellipse fit
        continue;
      //
      // Alloc memory for contour point set.
      PointArray = (CvPoint*)malloc( count*sizeof(CvPoint) );
      PointArray2D32f= (CvPoint2D32f*)malloc( count*sizeof(CvPoint2D32f) );
      //
      // Get contour point set.
      cvCvtSeqToArray(cont, PointArray, CV_WHOLE_SEQ);
      //
      // Convert CvPoint set to CvBox2D32f set.
      for(i = 0; i < count; i++)
      {
        PointArray2D32f[i].x = (float)PointArray[i].x;
        PointArray2D32f[i].y = (float)PointArray[i].y;
      }
      //
      // Fits ellipse to current contour.
      cvFitEllipse(PointArray2D32f, count, &ell[ellCnt]);
      // test the esitimated ellipse box for size
      eMaj = fmax(ell[ellCnt].size.width, ell[ellCnt].size.height); // major axis
      eMin = fmin(ell[ellCnt].size.width, ell[ellCnt].size.height); // major axis
      ratio = eMin/eMaj;
      printf("Maj=%.1f min=%.1f ratio=%.3f lim=%.3f, maxsize=%d, minsize=%d\n", eMaj, eMin, ratio, ratioLim, maxSize, minSize);
      if (eMaj < maxSize and eMin > minSize and ratio > ratioLim and eMin > varMinSize->getInt()) // and count > ePixCnt)
      { // an ellipse is found, so save its position and size
        //
        if (ellImg != NULL)
        { // debug - Draw current contour.
          // void cvDrawContours(CvArr*, CvSeq*, CvScalar, CvScalar, int, int, int)
          cvDrawContours(ellImg->cvArr(),cont,CV_RGB(255,255,255),CV_RGB(255,255,255),0,1,8 /*,cvPoint(0,0)*/);
          //
          // Convert ellipse data from float to integer representation.
          center.x = cvRound(ell[ellCnt].center.x);
          center.y = cvRound(ell[ellCnt].center.y);
          size.width = cvRound(ell[ellCnt].size.width * 0.5);
          size.height = cvRound(ell[ellCnt].size.height * 0.5);
          angle = ell[ellCnt].angle;
          //
          // Draw ellipse.
          cvEllipse(ellImg->cvArr(), center, size,
                    angle, 0, 360,
                    CV_RGB(0,0,255), 1, CV_AA, 0);
        }
        if (ellCnt < MAX_ELL_CNT)
          ellCnt++;
      }
      // Free allocated memory
      free(PointArray);
      free(PointArray2D32f);
    }
    return ellCnt > 0;
  }
  /**
   filter the image.
   Source and destination image may be the same.
   \param src is the source image
   \param dst is the destination image */
  void filterMask(UImage * src, UImage * dst)
  {
    UImagePool * imgPool;
    UImage *img1;
    int iterations = 1;
    //
    imgPool = (UImagePool*)getStaticResource("imgPool", true);
    img1 = imgPool->getImage(varPoolImg->getInt() + 5, true);
    img1->copyMeta(src, true);
    if (src != dst)  
      dst->copyMeta(src, true);
    //
    cvErode( src->cvArr(), img1->cvArr(), NULL, iterations);
    cvDilate( img1->cvArr(), dst->cvArr(), NULL, iterations);
  }

  /**
   * Find the best of the estimated ball candidate and calculate the ball position
   * relalive to the robot.
   * Expect estimate ellipses that may be balls in the array 'ell' with ellCnt ellipses found.
   * The result is converted to first camera coordinates using the camera focal length and a known ball size.
   * There is no radial error correction applied, so some error must be expected.
   * The camera coordinates are transformed to robot coordinates using a known camera pose (from the camera structure).
   * The result is saved in the 'ballPos' variable in meters relative to the robot.
   * If the ball is close to the edge of the image and partially out of the image, then one of the flags isOutL or isOutR is set to true.
   * \param cam is a pointer to a camera device with camera position and lens parameters
   * \param ellImg optional image for debug paint, emmits also more printout to console
   * \param ballDiam is the known diameter of the ball (in meter)
   * \returns true if possible - and if true, then the ballPos, isOutL and isOutR is set. */
  bool calculateBallPosition(UCamPush * cam, UImage * ellImg, double ballDiam)
  {
    bool result;
    double bestDiameter = 4.0; // in pixels
    int bestIdx = -1;
    CvBox2D32f * bestEll = NULL;
    int i;
    UPosition posCam;   // position of ball in camera coordinates
    double focalLength; // in pixels at current resolution
    double hx, hy;      // centre of image (in pixels)
    UPosRot camPose;
    UMatrix4 mCamPose;  // matrix for conversion to robot coordinates
    CvScalar red = CV_RGB(255, 100, 100);
    CvFont font;
    const int MSL = 50;
    char s[MSL];
    float eMaj, eMin;
    //
    result = (ellCnt > 0) and (cam != NULL);
    if (result)
    { // find best
      if (ellImg != NULL)
        printf("Found %d balls in image\n", ellCnt);
      // find the best (largest) ellipse
      for (i = 0; i < ellCnt; i++)
      {
        eMaj = fmax(ell[i].size.width, ell[i].size.height); // major axis
        if (eMaj > bestDiameter)
        {
          bestEll = &ell[i];
          bestDiameter = eMaj;
          bestIdx = i;
        }
        if (ellImg != NULL)
        { // debug mode
          eMin = fmin(ell[i].size.width, ell[i].size.height); // minor axis
          printf(" - %d size %3.1fmajor %3.1fminor (angle %3.1fdeg) at %3.1fx %3.1fy (pixels)\n", i,
                eMaj, eMin, ell[i].angle, ell[i].center.x, ell[i].center.y);
        }
      }
      result = bestIdx >= 0;
    }
    if (result)
    {// get focal length in pixels from camera
      focalLength = cam->getCamPar()->getFocalLength();
      // calculate distance from camera to ball
      posCam.x = focalLength / bestDiameter * ballDiam;
      // and to the right of the camera axis (opporsite x in pixels coordinates)
      hx = cam->getCamPar()->getHx();
      posCam.y = (hx - bestEll->center.x) / focalLength * posCam.x;
      // and the distance above the camera axis
      hy = cam->getCamPar()->getHy();
      posCam.z = (hy - bestEll->center.y) / focalLength * posCam.x;
      //
      // the position in camera coordinates now need to be converted to
      // robot coordinates, based on the known camera position on the robot
      // this is the 3D position of camera and orientation - pan, tilt and roll
      camPose = cam->getPosRot();
      // these 6 values (x, y, z, Omega, Phi, Kappa) can be expressed as a
      // 4x4 matrix to convert positions in cam coordinates to robot coordinates
      // (the function is intended to get the matrix from robot to map coordinates,
      // but it works equally well from camera to robot)
      mCamPose = camPose.getRtoMMatrix();
      // now we can get the ball center position in robot coordinates
      ballPos = mCamPose * posCam;
      // 
      if (ellImg != NULL)
      { // debug mode
        ballPos.print("Ball position");
        //
        // paint best position in image as a red cross
        cvLine(ellImg->cvArr(),
                    cvPoint((int)bestEll->center.x - 15, (int)bestEll->center.y),
                    cvPoint((int)bestEll->center.x + 15, (int)bestEll->center.y),
                    red, 1, CV_AA, 0);
        cvLine(ellImg->cvArr(),
                    cvPoint((int)bestEll->center.x, (int)bestEll->center.y - 15),
                    cvPoint((int)bestEll->center.x, (int)bestEll->center.y + 15),
                    red, 1, CV_AA, 0);
        cvInitFont( &font, CV_FONT_HERSHEY_PLAIN,
                  1.0, 1.0, 0.0, 1, 8);
        snprintf(s, MSL, "%.2fm", posCam.x);
        cvPutText(ellImg->cvArr(), s, cvPoint((int)bestEll->center.x + 4, (int)bestEll->center.y - 4), &font, red);
      }
    }
    else if (ellImg != NULL)
    { // debug mode
      printf("No ball found\n");
    }
    return result;
  }
  /**
  Get number of balls found in image */
  int getCnt()
  { return ellCnt; };
  /**
  Get position of this estimated ball position
  \returns the position as a UPosition structure */
  UPosition getPos()
  { return ballPos; };
  /**
  Get the ball size - from global variable store */
  double getBallSize()
  { return varBallSize->getValued(); };

  /**
  Make the variables that will be available to other plugins */
  void createBaseVar()
  {
    varPoolImg = addVar("poolImg", 45.0, "d", "(r/w) first image pool number to use");
    varRedLim      = addVar("redLim", "105 137 129 180", "d",
                        "(r/w) UV range range for red ball in YUV image"
                        "{uMin, uMax, vMin, vMax} range [0-255]");
    varRedPos   = addVar("redPos", "2.0 -1.0 0.0", "3d",
                        "(r) Position of ball in coordinates relative to robot");
    varRedTime  = addVar("redTime", 0.0, "d", "(r) time of red ball position");
    varRedCnt   = addVar("redCnt", 0.0, "d", "(r) Number of red balls found in last image");
    varBlueLim      = addVar("blueLim", "115 200 95 132", "d",
                        "(r/w) UV range range for blue ball in YUV image"
                        "{uMin, uMax, vMin, vMax} range [0-255]");
    varBluePos   = addVar("bluePos", "2.0 -1.0 0.0", "3d",
                        "(r) Position of ball in coordinates relative to robot");
    varBlueTime  = addVar("blueTime", 0.0, "d", "(r) time of blue ball position");
    varBlueCnt   = addVar("blueCnt", 0.0, "d", "(r) Number of blue balls found in last image");
    varBallSize = addVar("BallSize", 0.12, "d", "(r/w) Size of the ball (diameter [m])");
    varTopLine = addVar("topLine", 75, "d", "(r/w) is the topmost line that could be a ball on the floor.");
    varMajMinRatio = addVar("majMinRatio", 0.3, "d", "(r/w) minimum ration between major and minor axis of fitted ellipse");
    varMaxSize = addVar("maxSize", 150.0, "d", "(r/w) Is maximum size of ball in pixels (major exis)");
    varMinSize = addVar("minSize", 20.0, "d", "(r/w) Is maximum size of ball in pixels (minor exis)");
    varFac = addVar("fac", 2.0, "d", "Factor in the extraction process - see code to find out where");
  }
  /**
  * copy ball position to polygon plugin, so that it can be shown by client in top-view image
  * \param centerPos is center position of ball on ground
  * \param isBlue is color of ball
  * \param size is size in meters of ball */
  void makeDisplayPolygon(UPosition * centerPos, bool isBlue, double size, UTime imgTime)
  {
    UResPoly * resPoly = (UResPoly *) getStaticResource("poly", false, false);
    UResPoseHist * resOdo =  (UResPoseHist *) getStaticResource("odoPose", false, true);
    UPolyItem * poly;
    const int MSL = 64;
    char s[MSL];
    UPosition pb, pbc;
    UPose odoPose;
    double angle = 0;
    //
    if (resPoly != NULL)
    { // polygon plugin is loaded, so make polygon circle
      // make name
      if (isBlue)
        snprintf(s, MSL, "ballBlue-1");
      else
        snprintf(s, MSL, "ballRed-1");
      // get polygon - may be there already
      poly = resPoly->getItem(s);
      if (poly == NULL)
        // if not then add
        poly = resPoly->add(s);
      pbc = ballPos;
      if (resOdo != NULL)
      { // odometry plugin is loaded
        odoPose = resOdo->getPoseAtTime(imgTime);
        // get ball position in odometry coordinates
        pbc = odoPose.getPoseToMap(ballPos);
      }
      if (poly != NULL)
      { // set polygon to new size and color
        poly->lock();
        poly->clear();
        poly->cooSys = 0; // coordinate system is odometry
        poly->setAsPolygon(); // not a polyline
        if (isBlue)
          poly->setColor("bd-dd");
        else // is red
          poly->setColor("rd-dd");
        for (int i = 0; i < 8; i++)
        { // add a point for every 45 degrees
          pb.x = size / 2.0 * cos(angle) + pbc.x;
          pb.y = size / 2.0 * sin(angle) + pbc.y;
          pb.z = size / 2.0;
          poly->add(&pb);
          angle += M_PI * 2.0 / 8.0;
        }
        poly->updateTime = imgTime;
        resPoly->gotNewData();
        poly->unlock();
      }
    }
  };
  //
private:
  static const int MAX_ELL_CNT = 100;
  /// resulting ellipses found in image
  CvBox2D32f ell[MAX_ELL_CNT];
  /// number of ellipses found
  int ellCnt;
  /// most likely ball position in robot coordinates
  UPosition ballPos;
  /// size of source image in pixels
  int imgSizeH;
  /// size of source image in pixels
  int imgSizeW;
  /// pointer to limiting red values redMin, redMax, greenMin, greenMax
  UVariable * varPoolImg;
  /// pointer to limiting red values redMin, redMax, greenMin, greenMax
  UVariable * varRedLim;
  /// pointer to ball position {x,y,z}
  UVariable * varRedPos;
  /// pointer to time of detection
  UVariable * varRedTime;
  /// pointer to number of balls found in image
  UVariable * varRedCnt;
  /// pointer to limiting red values uMin, uMax, vMin, vMax
  UVariable * varBlueLim;
  /// pointer to ball position {x,y,z}
  UVariable * varBluePos;
  /// pointer to time of detection
  UVariable * varBlueTime;
  /// pointer to number of balls found in image
  UVariable * varBlueCnt;
  /// pointer to known size of ball
  UVariable * varBallSize;
  /// topmost line in image
  UVariable * varTopLine;
  /// minimum ratio metween major and minor axis of ellipse
  UVariable * varMajMinRatio;
  /// Maximum size for an estimated ball
  UVariable * varMaxSize;
  /// Maximum size for an estimated ball
  UVariable * varMinSize;
  /// Maximum size for an estimated ball
  UVariable * varFac;
};


#ifdef LIBRARY_OPEN_NEEDED
UFunctionBase * createFunc()
{ // called by server to create an object of this type
  /** replace 'UFuncBall' with your classname, as used in the headerfile */
  return new UFuncBall();
}
#endif
