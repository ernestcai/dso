/**
* This file is part of DSO.
* 
* Copyright 2016 Technical University of Munich and Intel.
* Developed by Jakob Engel <engelj at in dot tum dot de>,
* for more information see <http://vision.in.tum.de/dso>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* DSO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DSO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DSO. If not, see <http://www.gnu.org/licenses/>.
*/



#include <stdio.h>
#include "util/settings.h"

//#include <GL/glx.h>
//#include <GL/gl.h>
//#include <GL/glu.h>

#include <pangolin/pangolin.h>
#include "KeyFrameDisplay.h"
#include "FullSystem/HessianBlocks.h"
#include "FullSystem/ImmaturePoint.h"
#include "util/FrameShell.h"



namespace dso
{
namespace IOWrap
{


//    template <int ppp>
//	InputPointSparse<ppp>::InputPointSparse() {
//		// observations = new std::vector<Observation>();
//	}
//
//	template <int ppp>
//	InputPointSparse<ppp>::~InputPointSparse() {
//		// delete observations;
//	}

KeyFrameDisplay::KeyFrameDisplay()
{
	originalInputSparse = 0;
	numSparseBufferSize=0;
	numSparsePoints=0;

	id = 0;
	active= true;
	camToWorld = SE3();

	needRefresh=true;

	my_scaledTH =1e10;
	my_absTH = 1e10;
	my_displayMode = 1;
	my_minRelBS = 0;
	my_sparsifyFactor = 1;

	numGLBufferPoints=0;
	bufferValid = false;
}
void KeyFrameDisplay::setFromF(FrameShell* frame, CalibHessian* HCalib)
{
	id = frame->id;
	fx = HCalib->fxl();
	fy = HCalib->fyl();
	cx = HCalib->cxl();
	cy = HCalib->cyl();
	width = wG[0];
	height = hG[0];
	fxi = 1/fx;
	fyi = 1/fy;
	cxi = -cx / fx;
	cyi = -cy / fy;
	camToWorld = frame->camToWorld;
    filename = frame->filename;
	needRefresh=true;
}

void KeyFrameDisplay::setFromKF(FrameHessian* fh, CalibHessian* HCalib)
{
	setFromF(fh->shell, HCalib);

	// add all traces, inlier and outlier points.
//	int npoints = 	fh->immaturePoints.size() +
//					fh->pointHessians.size() +
//					fh->pointHessiansMarginalized.size() +
//					fh->pointHessiansOut.size();
//	int npoints = fh->pointHessians.size() +
//					 fh->pointHessiansMarginalized.size() +
//					 fh->pointHessiansOut.size();
    int npoints = fh->pointHessiansMarginalized.size();

	if(numSparseBufferSize < npoints)
	{
		if(originalInputSparse != 0) {
            for (int i = 0; i < numSparseBufferSize; i++){
                delete originalInputSparse[i].observations;
            }
            delete[] originalInputSparse;
        }

		numSparseBufferSize = npoints+100;
        originalInputSparse = new InputPointSparse<MAX_RES_PER_POINT>[numSparseBufferSize];
        for (int i = 0; i < numSparseBufferSize; i++){
            originalInputSparse[i].observations = new std::vector<Observation>;
        }
	}

    InputPointSparse<MAX_RES_PER_POINT>* pc = originalInputSparse;
	numSparsePoints=0;
//	for(ImmaturePoint* p : fh->immaturePoints)
//	{
//		for(int i=0;i<patternNum;i++)
//			pc[numSparsePoints].color[i] = p->color[i];
//
//		pc[numSparsePoints].u = p->u;
//		pc[numSparsePoints].v = p->v;
//		pc[numSparsePoints].idpeth = (p->idepth_max+p->idepth_min)*0.5f;
//		pc[numSparsePoints].idepth_hessian = 1000;
//		pc[numSparsePoints].relObsBaseline = 0;
//		pc[numSparsePoints].numGoodRes = 1;
//		pc[numSparsePoints].status = 0;
//		numSparsePoints++;
//	}

//	for(PointHessian* p : fh->pointHessians)
//	{
//		for(int i=0;i<patternNum;i++)
//			pc[numSparsePoints].color[i] = p->color[i];
//		pc[numSparsePoints].u = p->u;
//		pc[numSparsePoints].v = p->v;
//		pc[numSparsePoints].idpeth = p->idepth_scaled;
//		pc[numSparsePoints].relObsBaseline = p->maxRelBaseline;
//		pc[numSparsePoints].idepth_hessian = p->idepth_hessian;
//		pc[numSparsePoints].numGoodRes =  0;
//		pc[numSparsePoints].status=1;
//
//		numSparsePoints++;
//	}

	for(PointHessian* p : fh->pointHessiansMarginalized)
	{
		for(int i=0;i<patternNum;i++)
			pc[numSparsePoints].color[i] = p->color[i];
		pc[numSparsePoints].u = p->u;
		pc[numSparsePoints].v = p->v;
		pc[numSparsePoints].idpeth = p->idepth_scaled;
		pc[numSparsePoints].relObsBaseline = p->maxRelBaseline;
		pc[numSparsePoints].idepth_hessian = p->idepth_hessian;
		pc[numSparsePoints].numGoodRes =  0;
		pc[numSparsePoints].status=2;

        // save observation on the host frame
        addObservation(
                &pc[numSparsePoints],
                Observation(p->host->shell->id,  Eigen::Vector2f(p->u,p->v))
        );
//        pc[numSparsePoints].observations->push_back(
//                Observation(p->host->shell->id,  Eigen::Vector2f(p->u,p->v))
//        );

        // save observation on all target frames
        for (auto ob : p->residuals){
            // make sure all host of residual are of the point host
            assert(p->host->shell->id == ob->host->shell->id);
			if (ob->target_frame_id != 0){
                addObservation(
                        &pc[numSparsePoints],
                        Observation(ob->target_frame_id, ob->projectedTo[4])
                );
                // std::cerr << "observation target_frame_id: " <<  ob->target_frame_id << std::endl;
//                pc[numSparsePoints].observations->push_back(
//                        Observation(ob->target_frame_id, ob->projectedTo[4])      // use the center point
//                );
            }
        }

		numSparsePoints++;
	}

//	for(PointHessian* p : fh->pointHessiansOut)
//	{
//		for(int i=0;i<patternNum;i++)
//			pc[numSparsePoints].color[i] = p->color[i];
//		pc[numSparsePoints].u = p->u;
//		pc[numSparsePoints].v = p->v;
//		pc[numSparsePoints].idpeth = p->idepth_scaled;
//		pc[numSparsePoints].relObsBaseline = p->maxRelBaseline;
//		pc[numSparsePoints].idepth_hessian = p->idepth_hessian;
//		pc[numSparsePoints].numGoodRes =  0;
//		pc[numSparsePoints].status=3;
//		numSparsePoints++;
//	}
	assert(numSparsePoints <= npoints);

	camToWorld = fh->PRE_camToWorld;
	needRefresh=true;
}


KeyFrameDisplay::~KeyFrameDisplay()
{
	if(originalInputSparse != 0)
		delete[] originalInputSparse;
}

pcl::PointCloud<pcl::PointXYZ> KeyFrameDisplay::getPC() {

    pcl::PointCloud<pcl::PointXYZ> cloud;

    for(int i=0;i<numSparsePoints;i++)
    {
        if(originalInputSparse[i].idpeth < 0) continue;


        float depth = 1.0f / originalInputSparse[i].idpeth;
        float depth4 = depth*depth; depth4*= depth4;
        float var = (1.0f / (originalInputSparse[i].idepth_hessian+0.01));

        if(var * depth4 > my_scaledTH)
            continue;

        if(var > my_absTH)
            continue;

        if(originalInputSparse[i].relObsBaseline < my_minRelBS)
            continue;


        //for(int pnt=0;pnt<patternNum;pnt++)
        // only need one point here, set pnt=4, the central point
        for(int pnt=4;pnt<5;pnt++)
        {
            if(my_sparsifyFactor > 1 && rand()%my_sparsifyFactor != 0) continue;
            int dx = patternP[pnt][0];
            int dy = patternP[pnt][1];

            float pos_x = ((originalInputSparse[i].u+dx)*fxi + cxi) * depth;
            float pos_y = ((originalInputSparse[i].v+dy)*fyi + cyi) * depth;
            float pos_z = depth*(1 + 2*fxi * (rand()/(float)RAND_MAX-0.5f));
            pcl::PointXYZ point(pos_x,pos_y,pos_z);
            cloud.push_back(point);


// deal with color
//            tmpColorBuffer[vertexBufferNumPoints][0] = originalInputSparse[i].color[pnt];
//            tmpColorBuffer[vertexBufferNumPoints][1] = originalInputSparse[i].color[pnt];
//            tmpColorBuffer[vertexBufferNumPoints][2] = originalInputSparse[i].color[pnt];
        }
    }

    auto cam_r = camToWorld.rotationMatrix();
    auto cam_t = camToWorld.translation();
    Eigen::Matrix4f cam(4,4);
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            cam(i,j) = cam_r(i,j);
        }
    }
    for (int i = 0; i < 3; i ++){
        cam(i,3) = cam_t(i);
    }
    cam(3,3) = 1;

    pcl::PointCloud<pcl::PointXYZ> cloud_transformed;
    pcl::transformPointCloud(cloud,cloud_transformed,cam);

    return cloud_transformed;
}

bool KeyFrameDisplay::refreshPC(bool canRefresh, float scaledTH, float absTH, int mode, float minBS, int sparsity)
{
	if(canRefresh)
	{
		needRefresh = needRefresh ||
				my_scaledTH != scaledTH ||
				my_absTH != absTH ||
				my_displayMode != mode ||
				my_minRelBS != minBS ||
				my_sparsifyFactor != sparsity;
	}

	if(!needRefresh) return false;
	needRefresh=false;

	my_scaledTH = scaledTH;
	my_absTH = absTH;
	my_displayMode = mode;
	my_minRelBS = minBS;
	my_sparsifyFactor = sparsity;


	// if there are no vertices, done!
	if(numSparsePoints == 0)
		return false;

	// make data
	Vec3f* tmpVertexBuffer = new Vec3f[numSparsePoints*patternNum];
	Vec3b* tmpColorBuffer = new Vec3b[numSparsePoints*patternNum];
	int vertexBufferNumPoints=0;

	for(int i=0;i<numSparsePoints;i++)
	{
		/* display modes:
		 * my_displayMode==0 - all pts, color-coded
		 * my_displayMode==1 - normal points
		 * my_displayMode==2 - active only
		 * my_displayMode==3 - nothing
		 */

		if(my_displayMode==1 && originalInputSparse[i].status != 1 && originalInputSparse[i].status!= 2) continue;
		if(my_displayMode==2 && originalInputSparse[i].status != 1) continue;
		if(my_displayMode>2) continue;

		if(originalInputSparse[i].idpeth < 0) continue;


		float depth = 1.0f / originalInputSparse[i].idpeth;
		float depth4 = depth*depth; depth4*= depth4;
		float var = (1.0f / (originalInputSparse[i].idepth_hessian+0.01));

		if(var * depth4 > my_scaledTH)
			continue;

		if(var > my_absTH)
			continue;

		if(originalInputSparse[i].relObsBaseline < my_minRelBS)
			continue;


		for(int pnt=0;pnt<patternNum;pnt++)
		{

			if(my_sparsifyFactor > 1 && rand()%my_sparsifyFactor != 0) continue;
			int dx = patternP[pnt][0];
			int dy = patternP[pnt][1];

			tmpVertexBuffer[vertexBufferNumPoints][0] = ((originalInputSparse[i].u+dx)*fxi + cxi) * depth;
			tmpVertexBuffer[vertexBufferNumPoints][1] = ((originalInputSparse[i].v+dy)*fyi + cyi) * depth;
			tmpVertexBuffer[vertexBufferNumPoints][2] = depth*(1 + 2*fxi * (rand()/(float)RAND_MAX-0.5f));



			if(my_displayMode==0)
			{
				if(originalInputSparse[i].status==0)
				{
					tmpColorBuffer[vertexBufferNumPoints][0] = 0;
					tmpColorBuffer[vertexBufferNumPoints][1] = 255;
					tmpColorBuffer[vertexBufferNumPoints][2] = 255;
				}
				else if(originalInputSparse[i].status==1)
				{
					tmpColorBuffer[vertexBufferNumPoints][0] = 0;
					tmpColorBuffer[vertexBufferNumPoints][1] = 255;
					tmpColorBuffer[vertexBufferNumPoints][2] = 0;
				}
				else if(originalInputSparse[i].status==2)
				{
					tmpColorBuffer[vertexBufferNumPoints][0] = 0;
					tmpColorBuffer[vertexBufferNumPoints][1] = 0;
					tmpColorBuffer[vertexBufferNumPoints][2] = 255;
				}
				else if(originalInputSparse[i].status==3)
				{
					tmpColorBuffer[vertexBufferNumPoints][0] = 255;
					tmpColorBuffer[vertexBufferNumPoints][1] = 0;
					tmpColorBuffer[vertexBufferNumPoints][2] = 0;
				}
				else
				{
					tmpColorBuffer[vertexBufferNumPoints][0] = 255;
					tmpColorBuffer[vertexBufferNumPoints][1] = 255;
					tmpColorBuffer[vertexBufferNumPoints][2] = 255;
				}

			}
			else
			{
				tmpColorBuffer[vertexBufferNumPoints][0] = originalInputSparse[i].color[pnt];
				tmpColorBuffer[vertexBufferNumPoints][1] = originalInputSparse[i].color[pnt];
				tmpColorBuffer[vertexBufferNumPoints][2] = originalInputSparse[i].color[pnt];
			}
			vertexBufferNumPoints++;


			assert(vertexBufferNumPoints <= numSparsePoints*patternNum);
		}
	}

	if(vertexBufferNumPoints==0)
	{
		delete[] tmpColorBuffer;
		delete[] tmpVertexBuffer;
		return true;
	}

	numGLBufferGoodPoints = vertexBufferNumPoints;
	if(numGLBufferGoodPoints > numGLBufferPoints)
	{
		numGLBufferPoints = vertexBufferNumPoints*1.3;
		vertexBuffer.Reinitialise(pangolin::GlArrayBuffer, numGLBufferPoints, GL_FLOAT, 3, GL_DYNAMIC_DRAW );
		colorBuffer.Reinitialise(pangolin::GlArrayBuffer, numGLBufferPoints, GL_UNSIGNED_BYTE, 3, GL_DYNAMIC_DRAW );
	}
	vertexBuffer.Upload(tmpVertexBuffer, sizeof(float)*3*numGLBufferGoodPoints, 0);
	colorBuffer.Upload(tmpColorBuffer, sizeof(unsigned char)*3*numGLBufferGoodPoints, 0);
	bufferValid=true;
	delete[] tmpColorBuffer;
	delete[] tmpVertexBuffer;


	return true;
}



void KeyFrameDisplay::drawCam(float lineWidth, float* color, float sizeFactor)
{
	if(width == 0)
		return;

	float sz=sizeFactor;

	glPushMatrix();

		Sophus::Matrix4f m = camToWorld.matrix().cast<float>();
		glMultMatrixf((GLfloat*)m.data());

		if(color == 0)
		{
			glColor3f(1,0,0);
		}
		else
			glColor3f(color[0],color[1],color[2]);

		glLineWidth(lineWidth);
		glBegin(GL_LINES);
		glVertex3f(0,0,0);
		glVertex3f(sz*(0-cx)/fx,sz*(0-cy)/fy,sz);
		glVertex3f(0,0,0);
		glVertex3f(sz*(0-cx)/fx,sz*(height-1-cy)/fy,sz);
		glVertex3f(0,0,0);
		glVertex3f(sz*(width-1-cx)/fx,sz*(height-1-cy)/fy,sz);
		glVertex3f(0,0,0);
		glVertex3f(sz*(width-1-cx)/fx,sz*(0-cy)/fy,sz);

		glVertex3f(sz*(width-1-cx)/fx,sz*(0-cy)/fy,sz);
		glVertex3f(sz*(width-1-cx)/fx,sz*(height-1-cy)/fy,sz);

		glVertex3f(sz*(width-1-cx)/fx,sz*(height-1-cy)/fy,sz);
		glVertex3f(sz*(0-cx)/fx,sz*(height-1-cy)/fy,sz);

		glVertex3f(sz*(0-cx)/fx,sz*(height-1-cy)/fy,sz);
		glVertex3f(sz*(0-cx)/fx,sz*(0-cy)/fy,sz);

		glVertex3f(sz*(0-cx)/fx,sz*(0-cy)/fy,sz);
		glVertex3f(sz*(width-1-cx)/fx,sz*(0-cy)/fy,sz);

		glEnd();
	glPopMatrix();
}


void KeyFrameDisplay::drawPC(float pointSize)
{

	if(!bufferValid || numGLBufferGoodPoints==0)
		return;


	glDisable(GL_LIGHTING);

	glPushMatrix();

		Sophus::Matrix4f m = camToWorld.matrix().cast<float>();
		glMultMatrixf((GLfloat*)m.data());

		glPointSize(pointSize);


		colorBuffer.Bind();
		glColorPointer(colorBuffer.count_per_element, colorBuffer.datatype, 0, 0);
		glEnableClientState(GL_COLOR_ARRAY);

		vertexBuffer.Bind();
		glVertexPointer(vertexBuffer.count_per_element, vertexBuffer.datatype, 0, 0);
		glEnableClientState(GL_VERTEX_ARRAY);
		glDrawArrays(GL_POINTS, 0, numGLBufferGoodPoints);
		glDisableClientState(GL_VERTEX_ARRAY);
		vertexBuffer.Unbind();

		glDisableClientState(GL_COLOR_ARRAY);
		colorBuffer.Unbind();

	glPopMatrix();
}


        int KeyFrameDisplay::printPoints(int starting_index, std::ofstream &myfile) {
            auto cam_r = camToWorld.rotationMatrix();
            auto cam_t = camToWorld.translation();
            Eigen::Matrix3d rotation_mat(3,3);
            for (int i = 0; i < 3; i++){
                for (int j = 0; j < 3; j++){
                    rotation_mat(i,j) = cam_r(i,j);
                }
            }

            Eigen::Quaterniond q_cam(rotation_mat);
            Eigen::Vector3d t_cam(cam_t);

            // print points
            int point_idx = 0;
            for(int i=0;i<numSparsePoints;i++)
            {
                if(originalInputSparse[i].idpeth < 0) continue;


                float depth = 1.0f / originalInputSparse[i].idpeth;
                float depth4 = depth*depth; depth4*= depth4;
                float var = (1.0f / (originalInputSparse[i].idepth_hessian+0.01));

                if(var * depth4 > my_scaledTH)
                    continue;

                if(var > my_absTH)
                    continue;

                if(originalInputSparse[i].relObsBaseline < my_minRelBS)
                    continue;


                //for(int pnt=0;pnt<patternNum;pnt++)
                // only need one point here, set pnt=4, the central point
                int dx = patternP[4][0];
                int dy = patternP[4][1];

                Eigen::Vector3d pos;
                pos(0) = ((originalInputSparse[i].u+dx)*fxi + cxi) * depth;
                pos(1) = ((originalInputSparse[i].v+dy)*fyi + cyi) * depth;
                pos(2) = depth*(1 + 2*fxi * (rand()/(float)RAND_MAX-0.5f));

                Eigen::Vector3d pos_new = cam_r * pos + cam_t;

                std::string output = "";
                output += "  - id: " + std::to_string(point_idx + starting_index) + "\n";
                output += "    pose:\n";
                output += "      - " + std::to_string(pos_new(0)) + "\n";
                output += "      - " + std::to_string(pos_new(1)) + "\n";
                output += "      - " + std::to_string(pos_new(2)) + "\n";
                output += "    observations:\n";
                for (Observation ob : (*originalInputSparse[i].observations)){
                    output += "      - kf: " + std::to_string(ob.frame_id) + "\n";
                    output += "        pixel:\n";
                    output += "          - "+ std::to_string(ob.projection(0)) + "\n";
                    output += "          - "+ std::to_string(ob.projection(1)) + "\n";
                }

                myfile << output;
                point_idx ++;
            }

            return point_idx;

        }

        void KeyFrameDisplay::addObservation(struct InputPointSparse<MAX_RES_PER_POINT> *pt, const Observation &ob) {
            for (int i = 0; i < (pt->observations->size()); i++){
                if (ob.frame_id == (pt->observations->at(i).frame_id)){
                    pt->observations->at(i) = ob;
                    return;
                }
            }
            pt->observations->push_back(ob);
        }

}
}
