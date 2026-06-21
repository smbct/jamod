#include <iostream>
#include <vector>

// prevent a clash between eigen and X11
#ifdef Success
#undef Success
#endif
#include <eigen3/Eigen/Geometry>

#include "qgl.h" // drawing routines
#include "tr_local.h" // definition of "extern backEndState_t	backEnd;" for entity data

void drawEntitiesBBox() {

	qglDepthRange( 0, 1 );
	
	int bbox_ind = 0;
	for(int entity_ind = 0; entity_ind < backEnd.refdef.num_entities; entity_ind ++) {

		// if(backEnd.refdef.entities[i].e.reType != RT_MODEL) {
		// 	continue;
		// }

		qhandle_t hModel = backEnd.refdef.entities[entity_ind].e.hModel;

		// get boundaries for this model
		vec3_t bounds1; vec3_t bounds2;
		R_ModelBounds(hModel, bounds1, bounds2);

		std::vector<Eigen::Vector3d> bbox_vertex(8);
		bbox_vertex[0] = Eigen::Vector3d(bounds1[0], bounds1[1], bounds1[2]);
		bbox_vertex[1] = Eigen::Vector3d(bounds1[0], bounds2[1], bounds1[2]);
		bbox_vertex[2] = Eigen::Vector3d(bounds2[0], bounds2[1], bounds1[2]);
		bbox_vertex[3] = Eigen::Vector3d(bounds2[0], bounds1[1], bounds1[2]);
		bbox_vertex[4] = Eigen::Vector3d(bounds1[0], bounds1[1], bounds2[2]);
		bbox_vertex[5] = Eigen::Vector3d(bounds1[0], bounds2[1], bounds2[2]);
		bbox_vertex[6] = Eigen::Vector3d(bounds2[0], bounds2[1], bounds2[2]);
		bbox_vertex[7] = Eigen::Vector3d(bounds2[0], bounds1[1], bounds2[2]);

		// rotate the bbox vertices by the current entity rotation matrix
		Eigen::Matrix3d entity_rot;
		for(int i = 0; i < 3; i++) { // row
			for(int j = 0; j < 3; j ++) {
				entity_rot(i,j) = backEnd.refdef.entities[entity_ind].e.axis[i][j];
			}
		}
		for(auto& vec: bbox_vertex) {
			vec = vec.transpose()*entity_rot;
		}

		// drawing part
		GL_Bind( tr.whiteImage );
		qglColor3f (1,1,1);
		GL_State( GLS_POLYMODE_LINE );

		qglPushMatrix();
		qglTranslatef(backEnd.refdef.entities[entity_ind].e.origin[0], backEnd.refdef.entities[entity_ind].e.origin[1], backEnd.refdef.entities[entity_ind].e.origin[2]);

		qglBegin (GL_LINES);
		for(int i = 0; i < 4; i ++) {
			int j = (i+1)%4;
			qglVertex3f (bbox_vertex[i].x(), bbox_vertex[i].y(), bbox_vertex[i].z());
			qglVertex3f (bbox_vertex[j].x(), bbox_vertex[j].y(), bbox_vertex[j].z());
		}

		for(int i = 4; i < 8; i ++) {
			int j = i < 7 ? i+1: 4;
			qglVertex3f (bbox_vertex[i].x(), bbox_vertex[i].y(), bbox_vertex[i].z());
			qglVertex3f (bbox_vertex[j].x(), bbox_vertex[j].y(), bbox_vertex[j].z());
		}

		for(int i = 0; i < 4; i ++) {
			qglVertex3f (bbox_vertex[i].x(), bbox_vertex[i].y(), bbox_vertex[i].z());
			qglVertex3f (bbox_vertex[i+4].x(), bbox_vertex[i+4].y(), bbox_vertex[i+4].z());
		}
		qglEnd ();

		qglPopMatrix();

		bbox_ind ++;
	}

}
