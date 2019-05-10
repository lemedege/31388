/***************************************************************************
 *   Copyright (C) 2005 by Christian Andersen and DTU                      *
 *   jca@oersted.dtu.dk                                                    *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#include <cstdlib>
#include <ulms4/ufunclaserbase.h>
#include <ugen4/udatabase.h>
#include <ugen4/upolygon.h>
#include <urob4/uresposehist.h>

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

/**
 * Class with one detect box */
class UNamedBox
{
public:
  /// polygon with detections
  UPolygon poly;
  /// corner polygon
  UPolygon cornerPolygon;
  /// area of box - if 0, then box is invalid
  double area;
  /// laser scanner pose
  UPosRot * laserPose;
  /// parent structure for global variables
  UVarPool * parentStruct;
  /// name of structure
  UVariable * varName;
  /// box definition in laser scanner coordinates
  UVariable * varBoxLimits;
  /// detections
  UVariable * varDetections;
  /// detection limits in robot coordinates
  UVariable * varDetectMinMaxRob;
  /// detection limits in selected coordinate system coordinates
  UVariable * varDetectMinMaxCoo;
  /// selected coordinate system
  UVariable * varCooSys;

public:
  UNamedBox()
  {
    poly.setSize(1000);
    cornerPolygon.setSize(4);
    //strncpy(name, "noname", MNL);
    //minXY.set(0,0,0);
    //maxXY.set(0,0,0);
    area = 0.0;
    parentStruct = NULL;
  }
  /// clear detections
  void clearPoly()
  {
    poly.clear();
  }
  /** set box
   * \param boxname name of box - used as polygon name
   * \param idx is the index or box number
   * \param x1,y1,x2,y2 is the corner coordinates in robot coordinates.
   * \param mRO coordinate conversion matrix to odometry coordinates */
  void setBox(const char * boxname, int idx, double x1, double y1, double x2, double y2, UMatrix4 * mRO)
  {
    UPosition minXY, maxXY;
    varName->setValues(boxname, 0, true);
    if (x1 < x2)
    {
      minXY.x = x1;
      maxXY.x = x2;
    }
    else
    {
      minXY.x = x2;
      maxXY.x = x1;
    }
    if (y1 < y2)
    {
      minXY.y = y1;
      maxXY.y = y2;
    }
    else
    {
      minXY.y = y2;
      maxXY.y = y1;
    }
    // save limits
    varBoxLimits->setValued(minXY.x, 0);
    varBoxLimits->setValued(minXY.y, 1);
    varBoxLimits->setValued(maxXY.x, 2);
    varBoxLimits->setValued(maxXY.y, 3);
    //
    poly.color[0] = (idx % 10) + '0';
    poly.color[1] = '1'; // two pixels wide
    poly.color[2] = '-'; // no marker
    poly.color[3] = 'd'; // default line style
    area = (maxXY.x - minXY.x) * (maxXY.y - minXY.y);
    // make also a corner polygon (for display)
    setCornersAsPolygon(mRO);
  }
  /** get box corners as polygon
   * \param boxpoly is the destination polygon.
   * \param mCC is the converion matrix to convert the coordinates (no conversion if NULL) */
  void setCornersAsPolygon(UMatrix4 * mRG)
  {
    UPosition pos[4], *po;
    // make all corners (in laser robot coordinates)
    pos[0].x = varBoxLimits->getDouble(0);
    pos[0].y = varBoxLimits->getDouble(1);
    pos[2].x = varBoxLimits->getDouble(2);
    pos[2].y = varBoxLimits->getDouble(3);
    pos[1].x = pos[0].x;
    pos[1].y = pos[2].y;
    pos[3].x = pos[2].x;
    pos[3].y = pos[0].y;
    cornerPolygon.clear();
    for (int i = 0; i < 4; i++)
    { // make polygon from corners
      cornerPolygon.add(pos[i]);
    }
    strncpy(cornerPolygon.color, poly.color, 8);
    cornerPolygon.setAsPolygon();
    po = cornerPolygon.getPoints();
    if (mRG != NULL)
    {
      for (int i = 0; i < cornerPolygon.getPointsCnt(); i++)
      {
        *po = *mRG * *po;
        po++;
      }
    }
  }
  //
  /** test if there are detections within boxex for this scan.
   * \param data is pointer to laser scan detections.
   * \param noPoly if true, then no displayable polygon is generated.
   * \param mRS is conversion matrix from Robot to Sensor coordinates.
   * \param mSR is conversion matrix from Sensor to Robot coordinates.
   * \param mRG is conversion matrix from Robot to selected (global) coordinate system.
   * \returns number of detections inside the defined box. */
  int doDetect(ULaserData * data, int polySys, UMatrix4 * mRS, UMatrix4 * mSR, UMatrix4 * mRG)
  {
    int cnt = 0;
    poly.clear();
    UPosition minXY, maxXY;
    // make all corners (in laser scanner coordinates)
    minXY.x = varBoxLimits->getDouble(0);
    minXY.y = varBoxLimits->getDouble(1);
    maxXY.x = varBoxLimits->getDouble(2);
    maxXY.y = varBoxLimits->getDouble(3);
    // convert to sensor coordinates
    if (area > 1e-4)
    {
      minXY = *mRS * minXY;
      maxXY = *mRS * maxXY;
      // detect
      for (int i = 0; i < data->getRangeCnt(); i++)
      { // range are stored as an integer in current units
        U2Dpos p;
        p = data->get2d(i);
        if (p.length() > 0.001)
        { // valid measurement
          if (p.x > minXY.x and p.x <= maxXY.x and
              p.y > minXY.y and p.y <= maxXY.y)
          { // a detect
            poly.add(p.x, p.y);
          }
        }
      }
      cnt = poly.getPointsCnt();
      if (polySys >= 0 and cnt > 0)
      { // make point cloud to a convex polygon
        UPolygon40 p40;
        UPosition * pt;
        if (cnt > 2)
        { // sort in right order and reduce to convex polygon
          poly.extractConvexTo(&p40);
          p40.copyPointsTo(&poly);
        }
        poly.setAsPolygon();
        pt = poly.getPoints();
        for (int i = 0; i < poly.getPointsCnt(); i++)
        { // convert to robot coordinates
          *pt = *mSR * *pt;
          pt++;
        }
      }
    }
    varDetections->setInt(cnt);
    // and limits
    if (cnt > 0)
      poly.getLimits(&minXY.x, &minXY.y, &maxXY.x, &maxXY.y);
    else
    {
      minXY.clear();
      maxXY.clear();
    }
    varDetectMinMaxRob->setDouble(minXY.x, 0);
    varDetectMinMaxRob->setDouble(minXY.y, 1);
    varDetectMinMaxRob->setDouble(maxXY.x, 2);
    varDetectMinMaxRob->setDouble(maxXY.y, 3);
    // now convert to selected coordinate system
    if (mRG != NULL)
    {
      if (cnt > 0)
      {
        UPosition * pt;
        pt = poly.getPoints();
        for (int i = 0; i < poly.getPointsCnt(); i++)
        { // convert to more global coordinates
          *pt = *mRG * *pt;
          pt++;
        }
        // and limits
        poly.getLimits(&minXY.x, &minXY.y, &maxXY.x, &maxXY.y);
      }
      varDetectMinMaxCoo->setDouble(minXY.x, 0);
      varDetectMinMaxCoo->setDouble(minXY.y, 1);
      varDetectMinMaxCoo->setDouble(maxXY.x, 2);
      varDetectMinMaxCoo->setDouble(maxXY.y, 3);
      // convert also the box frame
      setCornersAsPolygon(mRG);
    }
    return cnt;
  }
  /// create global variables
  void createGlobalVariables(UVarPool * vp)
  {
    parentStruct = vp;
    varName = vp->addVarA("name", "", "s", "(r) Name of the box");
    varBoxLimits = vp->addVarA("boxLimits", "0 0 0 0", "d", "(r) box limits [minX, minY, maxX, maxY] in robot coordinates");
    varDetections = vp->addVar("detectCnt", 0.0, "d", "(r) detection count in this box");
    varDetectMinMaxRob = vp->addVarA("detectMinMaxRob", "0 0 0 0", "d", "(r) detections limits [minX, minY, maxX, maxY] in robot coordinates");
    varDetectMinMaxCoo = vp->addVarA("detectMinMaxCoo", "0 0 0 0", "d", "(r) detections limits [minX, minY, maxX, maxY] in selected coordinate system");
    varCooSys = vp->addVar("cooSys", 0.0, "d", "(r/w) Selected coordinate system: 0=odometry, 1=GPS (UTM), 2=Map");
  }
};

///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////

/**
 * Class with detection boxes */

class UNamedBoxes
{
public:
  static const int MAX_BOXES = 30;
  /// the boxes
  UNamedBox boxes[MAX_BOXES];
  /// number of used boxes
  int boxesCnt;
  /// count of detection boxes
  UVariable * varBoxesCnt;
  /// last detection time
  UVariable * varDetectTime;
  /// count of detection attempts
  UVariable * varDetectCnt;
  /// detections in each box
  UVariable * varDetections;
  /// pointer to the plugin structure for global variables
  UVarPool * parentStruct;
  /// pose history for odometry
  UResPoseHist * phOdo;
  /// pose history for GPS (UTM)
  UResPoseHist * phUtm;
  /// pose history for map coordinates
  UResPoseHist * phMap;
public:
  /// constructor
  UNamedBoxes()
  {
    boxesCnt = 0;
    parentStruct = NULL;
    phOdo = NULL;
    phUtm = NULL;
    phMap = NULL;
  }
  /// create global variables
  void createGlobalVariables(UVarPool * vp)
  {
    parentStruct = vp;
    varBoxesCnt = parentStruct->addVar("boxCnt", 0, "d", "(r) Number of defined structures");
    varDetectTime = parentStruct->addVar("detectTime", 0, "t", "(r) time of last detect");
    varDetectCnt = parentStruct->addVar("detectCnt", 0, "d", "(r) number of detection appempts");
    varDetections = parentStruct->addVar("detections", 0, "d", "(r) number of detections in each box (-1 is inactive)");
  }
  /// find named box index
  int findNamedBox(const char * name)
  {
    for (int i = 0; i < boxesCnt; i++)
      if (strcmp(name, boxes[i].varName->getString()) == 0)
        return i;
    return -1;
  }
  /// add or modify a abox
  UNamedBox * addBox(const char * name, double x1, double y1, double x2, double y2)
  {
    int i;
    UNamedBox * bx = NULL;
    i = findNamedBox(name);
    if (i < 0)
    {
      for (i = 0; i < boxesCnt; i++)
        if (boxes[i].area < 1e-6)
          break;
    }
    if (i < MAX_BOXES and i >= boxesCnt)
    { // add a box
      const int MSL = 10;
      char s[MSL];
      UVarPool * bs;
      boxesCnt++;
      // update global variables
      varBoxesCnt->setInt(boxesCnt);
      varDetections->setInt(0, i, true);
      snprintf(s, MSL, "box%02d", i);
      bs = parentStruct->addStructLocal(s, "detection box details", false);
      boxes[i].createGlobalVariables(bs);
    }
    if (i < MAX_BOXES)
    {
      UMatrix4 mRG;
      if (phOdo != NULL)
        mRG = phOdo->getNewest().asMatrix4x4PtoMPos();
      bx = &boxes[i];
      bx->setBox(name, i, x1, y1, x2, y2, &mRG);
    }
    return bx;
  }
  /** test if there are detections within boxex for this scan
   * \param data is structure with laser data
   * \param noPoly if true, then no polygons are constructed from detections inside box
   * \param lasDev is pointer to source device (with position of laser scanner)
   * \returns number of boxes with detections */
  int doDetect(ULaserData * data, bool noPoly, ULaserDevice * lasDev, int polySys)
  {
    int cnt = 0;
    UMatrix4 mSR, mRS, mRG;
    UMatrix4 * pmRG = NULL;
    UResPoseHist * ph;
    mSR = lasDev->getDevicePose().getRtoMMatrix();
    mRS = lasDev->getDevicePose().getMtoRMatrix();
    for (int i = 0; i < boxesCnt; i++)
    {
      UNamedBox * nb = &boxes[i];
      int dc;
      nb->parentStruct->lock();
        if (polySys >= 0 and polySys <= 2 and polySys != nb->varCooSys->getInt())
          nb->varCooSys->setInt(polySys);
        switch (nb->varCooSys->getInt())
        { // select coordinate system reference
          case 1: ph = phUtm; break;
          case 2: ph = phMap; break;
          default: ph = phOdo; break;
        }
        if (ph != NULL)
        { // get conversion matrix
          UPose pr = ph->getPoseAtTime(data->getScanTime());
          mRG = pr.asMatrix4x4PtoMPos();
          pmRG = &mRG;
        }
        nb->clearPoly();
        dc = nb->doDetect(data, noPoly, &mRS, &mSR, pmRG);
        varDetections->setInt(dc, i);
        if (dc > 0)
          cnt++;
      nb->parentStruct->unlock();
    }
    varDetectCnt->add(1);
    varDetectTime->setTime(data->getScanTime());
    return cnt;
  }
};

///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
/**
 * Laserscanner function to demonstrate
 * simple laser scanner data handling and analysis
 * @author Christian Andersen
*/
class UFuncLaserBox : public UFuncLaserBase
{
public:
  /// array of detection boxes
  UNamedBoxes boxes;
  /// struct variables for detections
  UVarPool boxStructs[UNamedBoxes::MAX_BOXES];
  ///

public:
  /**
  Constructor */
  UFuncLaserBox()
  { // set the command (or commands) handled by this plugin
    setCommand("laserbox", "laserbox", "find laser scan detections in defined box(es)");
  }
  /**
   * Create any resources that this modules needs
   * This method is called after the resource is registred by the server. */
  virtual void createResources()
  {
    boxes.createGlobalVariables((UVarPool *)getVarPool());
  }
  /**
  Handle incomming command
  (intended for command separation)
  Must return true if the function is handled -
  otherwise the client will get a failed - reply */
  virtual bool handleCommand(UServerInMsg * msg, void * extra)
  {  // handle a plugin command
    const int MRL = 500;
    char reply[MRL];
    bool ask4help;
    const int MVL = 30;
    char value[MVL];
    ULaserData * data;
    //
    // check for parameters - one parameter is tested for - 'help'
    ask4help = msg->tag.getAttValue("help", value, MVL);
    if (ask4help)
    { // create the reply in XML-like (html - like) format
      sendHelpStart(msg, "inbox");
      sendText("--- available INBOX options\n");
      sendText("add=name x1=A y1=B x2=C y2=D");
      sendText("                add named box with limits A,B,C,D in robot coordinates\n");
      sendText("detect          Detect for objects in box\n");
      sendText("poly=S          make also result polygons in -1=none, 0=odo (default), 1=UPM, 2=Map coordinates\n");
      sendText("silent          send no (less) reply\n");
      sendText("fake=F          Fake some data 1=random, 2-4 a fake corridor\n");
      sendText("device=N        Laser device to use (see: SCANGET help)\n");
      sendText("see also: SCANGET and SCANSET\n");
      sendText("hej hej");
      sendHelpDone();
    }
    else
    { // do some action and send a reply
      int polySys = -2;
      bool silent;
      msg->tag.getAttInteger("poly", &polySys, 0);
      // test if silent option - i.e. no reply needed
      msg->tag.getAttBool("silent", &silent);
      // test add option
      if (msg->tag.getAttValue("add", value, MVL))
      { // get all needed parameters
        double x1, x2, y1, y2;
        bool gotX1, gotX2, gotY1, gotY2;
        gotX1 = msg->tag.getAttDouble("x1", &x1);
        gotX2 = msg->tag.getAttDouble("x2", &x2);
        gotY1 = msg->tag.getAttDouble("y1", &y1);
        gotY2 = msg->tag.getAttDouble("y2", &y2);
        if (gotX1 and gotX2 and gotY1 and gotY2)
        {
          UNamedBox * bx;
          bx = boxes.addBox(value, x1, y1, x2, y2);
          if (not silent)
          {
            if (bx != NULL)
              sendInfo("box added");
            else
              sendWarning("failed to add box");
          }
          if (polySys != -1 and bx != NULL)
          {
            snprintf(reply, MRL, "%s_Box", bx->varName->getString());
            sendToPolyPlugin(reply, &bx->cornerPolygon, 0);
          }
        }
        else
        {
          if (not silent)
            sendInfo("failed to add box, not enough parameters");
        }
      }
      // test detect option
      if (msg->tag.getAttValue("detect", NULL, 0))
      {
        ULaserDevice * lasDev; // laser device
        data = getScan(msg, (ULaserData*)extra, false, &lasDev);
        //
        if (data->isValid())
        { // make analysis for closest measurement
          int i;
          // double minAngle = 0.0; // degrees
          i = boxes.doDetect(data, polySys, lasDev, polySys);
          if (not silent and boxes.boxesCnt > 1)
          {
            snprintf(reply, MRL, "detections in %d boxes", i);
            sendInfo(reply);
          }
          if (not silent)
          { // send reply to client
            UNamedBox * nb;
            snprintf(reply, MRL, "boxCnt=\"%d\"", boxes.boxesCnt);
            sendStartTag(reply);
            for (int i= 0; i < boxes.boxesCnt; i++)
            {
              nb = &boxes.boxes[i];
              snprintf(reply, MRL, "<box idx=\"%d\" name=\"%s\" detectCnt=\"%d\" minX=\"%f\" minY=\"%f\" maxX=\"%f\" maxY=\"%f\"/>\n",
                i, nb->varName->getString(), nb->varDetections->getInt(), nb->varDetectMinMaxCoo->getDouble(0),nb->varDetectMinMaxCoo->getDouble(1),nb->varDetectMinMaxCoo->getDouble(2),nb->varDetectMinMaxCoo->getDouble(3));
              sendMsg(reply);
              snprintf(reply, MRL, "<laser  l0=\"%f\"    l1=\"%f\"  l2=\"%f\"  l3=\"%f\"/>\n",
                   nb->varDetectMinMaxCoo->getDouble(0),nb->varDetectMinMaxCoo->getDouble(1),nb->varDetectMinMaxCoo->getDouble(2),nb->varDetectMinMaxCoo->getDouble(3));
   // send this string as the reply to the client
              sendMsg(msg, reply);
            }
            sendEndTag();
          }
          if (polySys != -1)
          { // send polygons to polygon plugin
            UNamedBox * bx;
            const int MSL = 20;
            char s[MSL];
            for (int i = 0; i < boxes.boxesCnt; i++)
            {
              bx = &boxes.boxes[i];
              snprintf(s, MSL, "%s_Box", bx->varName->getString());
              sendToPolyPlugin(bx->varName->getString(), &bx->poly, 0);
              sendToPolyPlugin(s, &bx->cornerPolygon, 0);
            }
          }
        }
        else
          sendWarning(msg, "No scandata available");
      }
    }
    // return true if the function is handled with a positive result
    return true;
  }

  /**
   * set resource when new resource is loaded */
virtual bool setResource(UResBase * resource, bool remove)
{
  bool result = false;
  //
  if (resource->isA(UResPoseHist::getOdoPoseID()))
  {
    result = true;
    if (remove)
      boxes.phOdo = NULL;
    else if (boxes.phOdo != resource)
      boxes.phOdo = (UResPoseHist *)resource;
    else
      // not used
      result = false;
  }
  else if (resource->isA(UResPoseHist::getMapPoseID()))
  {
    result = true;
    if (remove)
      boxes.phMap = NULL;
    else if (boxes.phMap != resource)
      boxes.phMap = (UResPoseHist *)resource;
    else
      result = false;
  }
  else if (resource->isA(UResPoseHist::getUtmPoseID()))
  {
    result = true;
    if (remove)
      boxes.phUtm = NULL;
    else if (boxes.phMap != resource)
      boxes.phUtm = (UResPoseHist *)resource;
    else
      // not used
      result = false;
  }
  result &= UFuncLaserBase::setResource(resource, remove);
  return result;
}


private:
  /**
   * Send a polygon to the poly-plugin (eventually for display in client)
   * \param polyName name of polygon (will replace any existing polygon with this name)
   * \param poly  pointer to the polygon to be transferred
   * \param cooSys is coordinate system used - 0=odometry, 1=utm, 2=map */
  void sendToPolyPlugin(const char * polyName, UPolygon * poly, int cooSys)
  {
    UVariable vs; // name of polygon
    UVariable vr; // return value
    UVariable vCoo; // coordinate system
    UDataBase *dbr = &vr; // return structure array for call
    UVariable * par[3] = {&vs, (UVariable *) poly, &vCoo}; // parameter array
    bool isOK;
    int n = 1; // number of available return parameters
    //
    // set polygon coordinate system to odometry (0=odo, 1=utm, 3=map)
    vCoo.setInt(0);
    // set polygon name
    vs.setValues(polyName, 0, true);
    if (poly->getPointsCnt() == 0)
      // no data, so delete polygon
      isOK = callGlobalV("poly.del", "s", par, &dbr,  &n);
    else
      isOK = callGlobalV("poly.setPolygon", "scd", par, &dbr,  &n);
    if (not isOK or not vr.getBool())
      printf("UFuncLaserBox::sendToPolyPlugin: failed to 'poly.setPoly()'\n");
  }

};


#ifdef LIBRARY_OPEN_NEEDED

/**
 * This function is needed by the server to create a version of this plugin */
UFunctionBase * createFunc()
{ // create an object of this type
  /** replace 'UFuncNear' with your classname */
  return new UFuncLaserBox();
}
#endif

///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
