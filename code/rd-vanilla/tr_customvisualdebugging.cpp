#include <iostream>
#include <vector>
#include <fstream>
#include <algorithm>

// prevent a clash between eigen and X11
#ifdef Success
#undef Success
#endif
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>

#include "qgl.h" // drawing routines
#include "tr_local.h" // definition of "extern backEndState_t	backEnd;" for entity data

#include "tr_common.h" // for skeleton drawing

#include "../ghoul2/G2.h" // skeletong related definitions

extern mdxaBone_t worldMatrix;


/*!
 * \brief draw a bounding box around entities
 */
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


/////////////////////////////////////////////////////////////
// skeleton visual debugging code
/////////////////////////////////////////////////////////////

// local definitions borrowed from tr_ghoul2.cpp
// Note that the code of actual functions is not necessary here, only the definitions that are used in the skeleton drawing code

class CTransformBone {
		public:
		int				touchRender;
		mdxaBone_t		boneMatrix; //final matrix
		int				parent; // only set once
		int				touch; // for minimal recalculation
		CTransformBone();
};

struct SBoneCalc {
	int				newFrame;
	int				currentFrame;
	float			backlerp;
	float			blendFrame;
	int				blendOldFrame;
	bool			blendMode;
	float			blendLerp;
};

void G2_TransformBone(int index,CBoneCache &CB);
int G2_Find_Bone(CGhoul2Info *ghlInfo, boneInfo_v &blist, const char *boneName);


class CBoneCache {
	void EvalLow(int index);
	void SmoothLow(int index);
public:
	int					frameSize;
	const mdxaHeader_t	*header;
	const model_t		*mod;
	SBoneCalc *mBones;
	CTransformBone *mFinalBones;
	CTransformBone *mSmoothBones; // for render smoothing
	mdxaSkel_t **mSkels;
	int				mNumBones;
	boneInfo_v		*rootBoneList;
	mdxaBone_t		rootMatrix;
	int				incomingTime;
	int				mCurrentTouch;
	int				mCurrentTouchRender;
	int				mLastTouch;
	int				mLastLastTouch;
	bool			mSmoothingActive;
	bool			mUnsquash;
	float			mSmoothFactor;

	CBoneCache(const model_t *amod,const mdxaHeader_t *aheader);
	~CBoneCache ();
	SBoneCalc &Root();
	const mdxaBone_t &EvalUnsmooth(int index);
	const mdxaBone_t &Eval(int index);
	const inline mdxaBone_t &EvalRender(int index);
	bool WasRendered(int index);
	int GetParent(int index);
	CTransformBone *EvalFull(int index);
};



// all bones
// model_root
// pelvis
// Motion
// lfemurYZ
// lfemurX
// ltibia
// ltalus
// rfemurYZ
// rfemurX
// rtibia
// rtalus
// lower_lumbar
// upper_lumbar
// thoracic
// cervical
// cranium
// ceyebrow
// jaw
// lblip2
// leye
// rblip2
// ltlip2
// rtlip2
// reye
// rclavical
// rhumerus
// rhumerusX
// rradius
// rradiusX
// rhand
// r_d1_j1
// r_d1_j2
// r_d2_j1
// r_d2_j2
// r_d4_j1
// r_d4_j2
// rhang_tag_bone
// lclavical
// lhumerus
// lhumerusX
// lradius
// lradiusX
// lhand
// l_d4_j1
// l_d4_j2
// l_d2_j1
// l_d2_j2
// l_d1_j1
// l_d1_j2
// ltail
// rtail
// lhang_tag_bone
// face

std::vector<std::string> all_bones = { 
std::string("model_root"),
std::string("pelvis"),
std::string("model_root"),
std::string("pelvis"),
std::string("Motion"),
std::string("lfemurYZ"),
std::string("lfemurX"),
std::string("ltibia"),
std::string("ltalus"),
std::string("rfemurYZ"),
std::string("rfemurX"),
std::string("rtibia"),
std::string("rtalus"),
std::string("lower_lumbar"),
std::string("upper_lumbar"),
std::string("thoracic"),
std::string("cervical"),
std::string("cranium"),
std::string("ceyebrow"),
std::string("jaw"),
std::string("lblip2"),
std::string("leye"),
std::string("rblip2"),
std::string("ltlip2"),
std::string("rtlip2"),
std::string("reye"),
std::string("rclavical"),
std::string("rhumerus"),
std::string("rhumerusX"),
std::string("rradius"),
std::string("rradiusX"),
std::string("rhand"),
std::string("r_d1_j1"),
std::string("r_d1_j2"),
std::string("r_d2_j1"),
std::string("r_d2_j2"),
std::string("r_d4_j1"),
std::string("r_d4_j2"),
std::string("rhang_tag_bone"),
std::string("lclavical"),
std::string("lhumerus"),
std::string("lhumerusX"),
std::string("lradius"),
std::string("lradiusX"),
std::string("lhand"),
std::string("l_d4_j1"),
std::string("l_d4_j2"),
std::string("l_d2_j1"),
std::string("l_d2_j2"),
std::string("l_d1_j1"),
std::string("l_d1_j2"),
std::string("ltail"),
std::string("rtail"),
std::string("lhang_tag_bone"),
std::string("face"),
};


std::vector<std::string> keep_bones = { 
// std::string("model_root"),
// std::string("pelvis"),
// std::string("Motion"),

std::string("lfemurYZ"),
// std::string("lfemurX"),

std::string("ltibia"),
std::string("ltalus"),

std::string("rfemurYZ"),
// std::string("rfemurX"),

std::string("rtibia"),
std::string("rtalus"),

std::string("lower_lumbar"),
std::string("upper_lumbar"),

std::string("thoracic"),
std::string("cervical"),
std::string("cranium"),


std::string("rhumerus"),
std::string("rradius"),

// std::string("rhumerusX"),
// std::string("rradiusX"),

std::string("lhumerus"),
std::string("lradius"),

// std::string("lhumerusX"),
// std::string("lradiusX"),

std::string("rhand"),
std::string("lhand"),

// std::string("rhang_tag_bone"),
// std::string("lhang_tag_bone"),

std::string("rclavical"),
std::string("lclavical"),

std::string("face"),
};



std::vector<std::string> keep_axis = { 

// std::string("lfemurYZ"),
// std::string("lfemurX"),

// std::string("ltibia"),
// std::string("ltalus"),

// std::string("rfemurYZ"),
// std::string("rfemurX"),

// std::string("rtibia"),
// std::string("rtalus"),

// std::string("lower_lumbar"),
// std::string("upper_lumbar"),

// std::string("thoracic"),
// std::string("cervical"),
// std::string("cranium"),


std::string("rhumerus"),
std::string("rradius"),

std::string("lhumerus"),
std::string("lradius"),

std::string("rhand"),
std::string("lhand"),

// std::string("rclavical"),
// std::string("lclavical"),

// std::string("face"),
};

void drawMatrix(std::vector<float>& pos, const mdxaBone_t& matrix) {

	// 3x4 matrix
	// 

	// display local axis
	float vec[3], p1[3], p2[3];
	float scale = 7.;

	vec[0] = matrix.matrix[0][0]*scale; vec[1] = matrix.matrix[1][0]*scale; vec[2] = matrix.matrix[2][0]*scale;
	// vec[0] = scale; vec[1] = 0.; vec[2] = 0.;
	
	p2[0] = pos[0]+vec[0]; p2[1] = pos[1]+vec[1]; p2[2] = pos[2]+vec[2];
	
	qglLineWidth(3);
	qglBegin (GL_LINES);
	qglColor3f (1,0,0);
	qglVertex3fv (pos.data());
	qglVertex3fv (p2);
	qglEnd();

	vec[0] = matrix.matrix[0][1]*scale; vec[1] = matrix.matrix[1][1]*scale; vec[2] = matrix.matrix[2][1]*scale;
	// vec[0] = 0.; vec[1] = scale; vec[2] = 0.;
	p2[0] = pos[0]+vec[0]; p2[1] = pos[1]+vec[1]; p2[2] = pos[2]+vec[2];
	qglBegin (GL_LINES);
	qglColor3f (0,1,0);
	qglVertex3fv (pos.data());
	qglVertex3fv (p2);
	qglEnd();

	vec[0] = matrix.matrix[0][2]*scale; vec[1] = matrix.matrix[1][2]*scale; vec[2] = matrix.matrix[2][2]*scale;
	// vec[0] = 0.; vec[1] = 0.; vec[2] = scale;
	p2[0] = pos[0]+vec[0]; p2[1] = pos[1]+vec[1]; p2[2] = pos[2]+vec[2];
	qglBegin (GL_LINES);
	qglColor3f (0,0,1);
	qglVertex3fv (pos.data());
	qglVertex3fv (p2);
	qglEnd();

	qglLineWidth(1.);

}

// copy right to left
void copy_bone_matrix(mdxaBone_t& left, mdxaBone_t& right) {
	for(int i = 0; i < 3; i ++) {
		for(int j = 0; j < 4; j ++) {
			left.matrix[i][j] = right.matrix[i][j]; 
		}
	}
}

// void Inverse_Matrix(mdxaBone_t *src, mdxaBone_t *dest);

const static mdxaBone_t	identityMatrix = { {
		{ 0.0f, -1.0f, 0.0f, 0.0f },
		{ 1.0f, 0.0f, 0.0f, 0.0f },
		{ 0.0f, 0.0f, 1.0f, 0.0f }
} };

static std::vector<float> custom_vert;
static float testAngle = 0;
void drawSkeletons() {

	static float humerus_angle_0 = 0;
	static float humerus_angle_1 = 0;
	static float humerus_angle_2 = 0;

	humerus_angle_2 += 0.4;
	if(humerus_angle_2 > 360) {
		humerus_angle_2 -= 360;
	}

	humerus_angle_0 += 0.4;
	if(humerus_angle_0 > 360) {
		humerus_angle_0 -= 360;
	}

	qglDepthRange( -1000, -1000 );


	int model_entity_size = 0;

	for(int i = 0; i < backEnd.refdef.num_entities; i ++) {
		// if(backEnd.refdef.entities[i].e.reType == RT_MODEL) {
			model_entity_size += 1;
		// }
	}

	custom_vert.resize(24*model_entity_size, 0.);	
	
	static int i2 = 0;
	// cout << endl << endl;
	// cout << "*****************************************" << endl;
	// cout << "debug drawing surfaces !! " << i2 << endl;
	// cout << "*****************************************" << endl;
	// cout << endl << endl;
	i2 += 1;

	int bbox_ind = 0;
	for(int entity_ind = 0; entity_ind < backEnd.refdef.num_entities; entity_ind ++) {

		if(entity_ind > 1) {
			continue;
		}

		// if(backEnd.refdef.entities[i].e.reType != RT_MODEL) {
		// 	continue;
		// }

		// cout << "origin: " << backEnd.refdef.entities[entity_ind].e.origin[0] << " ,";
		// cout <<  backEnd.refdef.entities[entity_ind].e.origin[1] << ", ";
		// cout << backEnd.refdef.entities[entity_ind].e.origin[2] << endl;
		float p1[3], p2[3], p3[3], p4[3];

		// for(int j = 0; j < 3; j ++) {
		// 	p1[j] = backEnd.refdef.entities[entity_ind].e.origin[j];
		// 	p2[j] = backEnd.refdef.entities[entity_ind].e.origin[j];
		// 	p3[j] = backEnd.refdef.entities[entity_ind].e.origin[j];
		// 	p4[j] = backEnd.refdef.entities[entity_ind].e.origin[j];
		// }
		// p2[0] += 10;
		// p3[1] += 10;
		// p4[2] += 10;
		for(int j = 0; j < 3; j ++) {
			p1[j] = backEnd.refdef.entities[entity_ind].e.origin[j];

			p2[j] = backEnd.refdef.entities[entity_ind].e.origin[j];
			p2[j] += backEnd.refdef.entities[entity_ind].e.axis[0][j]*10;

			p3[j] = backEnd.refdef.entities[entity_ind].e.origin[j];
			p3[j] += backEnd.refdef.entities[entity_ind].e.axis[1][j]*10;

			p4[j] = backEnd.refdef.entities[entity_ind].e.origin[j];
			p4[j] += backEnd.refdef.entities[entity_ind].e.axis[2][j]*10;

		}

		// std::cout << "entity origin: " << p1[0] << " " << p1[1] << " " << p1[2] << std::endl;


		// skeleton ?
		if(backEnd.refdef.entities[entity_ind].e.ghoul2) {

			CGhoul2Info_v& ghoul2 = *backEnd.refdef.entities[entity_ind].e.ghoul2;

			// draw 3d local axis
			qglBegin (GL_LINES);
			qglColor3f (1,0,0);

			qglVertex3fv (p1);
			qglVertex3fv (p2);

			qglColor3f (0,1,0);
			qglVertex3fv (p1);
			qglVertex3fv (p3);

			qglColor3f (0,0,1);
			qglVertex3fv (p1);
			qglVertex3fv (p4);

			qglEnd ();

			// sort the ghoul 2 models so bolt ons get bolted to the right model
			int modelCount;
			int modelList[32];
			// G2_Sort_Models(ghoul2, modelList, &modelCount);
			// int i, j;

			// for (j=0; j<modelCount; j++)
			// {
			// 	i = modelList[j];

			// }

			// try to access the skeleton limbs
			vec3_t scale;
			mdxaBone_t retMatrix, matrix;

			// create a local entity matrix
			// vec3_t		axis[3]; and origin
			mdxaBone_t local_entity_matrix;
			for(int i = 0; i < 3; i ++) { // row
				for(int j = 0; j < 3; j ++) { // col
					local_entity_matrix.matrix[i][j] = backEnd.refdef.entities[entity_ind].e.axis[j][i];
				}
				local_entity_matrix.matrix[i][3] = backEnd.refdef.entities[entity_ind].e.origin[i];
			}

			if(ghoul2.IsValid() && ghoul2.size() > 0 && ghoul2[0].mBltlist.size() > 0)  {
				
				for(int model_ind = 0; model_ind < ghoul2.size(); model_ind ++) {

					mdxaBone_t test_mat;
					// float angle0 = 0., angle1 = 0., angle2 = 0.;
					// float angle0 = 140., angle1 = 200., angle2 = 60.;
					static float angle0 = 40.000;
					static float angle1 = 20.000;
					static float angle2 = -120.000;
					angle0 += 0.3;
					if(angle0 > 360) {
						angle0 -= 360;
					}
					angle2 += 0.3;
					if(angle2 > 360) {
						angle2 -= 360;
					}
					angle1 -= 0.3;
					if(angle1 < 0) {
						angle1 += 360;
					}
					// float angle0 = 20., angle1 = 70., angle2 = 0.;

					// float angle0 = 0., angle1 = 0., angle2 = 30.;

					Eigen::Matrix3f rot_from_euler_t;
					{
						Eigen::AngleAxisf X(angle2*M_PI/180., Eigen::Vector3f::UnitX()); // ROLL
						Eigen::AngleAxisf Y(angle0*M_PI/180., Eigen::Vector3f::UnitY()); // PITCH
						Eigen::AngleAxisf Z(angle1*M_PI/180., Eigen::Vector3f::UnitZ()); // YAW
						rot_from_euler_t = Eigen::Quaternionf(Z*Y*X).toRotationMatrix();
					}
					for(int i = 0; i < 3; i ++) {
						for(int j = 0; j < 3; j ++) {
							test_mat.matrix[i][j] = rot_from_euler_t(i,j);
						}
						test_mat.matrix[i][3] = 0;
					}

					// test_mat.matrix[0][0] = 1; test_mat.matrix[0][1] = 0; test_mat.matrix[0][2] = 0; 
					// test_mat.matrix[1][0] = 0; test_mat.matrix[1][1] = 1; test_mat.matrix[1][2] = 0; 
					// test_mat.matrix[2][0] = 0; test_mat.matrix[2][1] = 0; test_mat.matrix[2][2] = 1; 

					Eigen::Matrix3f m;
					for(int i = 0; i < 3; i ++) {
						for(int j = 0; j < 3; j++) {
							m(i,j) = test_mat.matrix[i][j];
						}
					}

					// read order from file
					// int order0 = 1, order1 = 0, order2 = 2;
					// ifstream order_file("order.txt");
					// if(order_file) {
					// 	order_file >> order0; order_file >> order1; order_file >> order2;
					// 	order_file.close();
					// }

					float humerus_angles[3];
					{
						Eigen::Matrix<float,3,1> res = m.eulerAngles(2,1,0);
						humerus_angles[YAW] = res(0)*180./M_PI;
						humerus_angles[PITCH] = res(1)*180./M_PI;
						humerus_angles[ROLL] = -res(2)*180./M_PI;
						Com_Printf("Extracted angles: %.3f %.3f %.3f\n", humerus_angles[PITCH], humerus_angles[YAW], humerus_angles[ROLL]);
						Com_Printf("\n\n");						
					}

					Eorientations first, second, third;
					int val1=1, val2=3, val3=2;
					std::ifstream pos_file("pos.txt");
					if(pos_file) {
						pos_file >> val1; pos_file >> val2; pos_file >> val3;
					}
					first = Eorientations(val1); second = Eorientations(val2); third = Eorientations(val3);
					pos_file.close();

					// POSITIVE_X=1
					// POSITIVE_Y=3
					// POSITIVE_Z=2
					// NEGATIVE_X=4
					// NEGATIVE_Y=6
					// NEGATIVE_Z=5

					// Com_Printf("debug POS and NEG axis: \n");
					// Com_Printf("%d %d %d\n", POSITIVE_X, POSITIVE_Y, POSITIVE_Z);
					// Com_Printf("%d %d %d\n", NEGATIVE_X, NEGATIVE_Y, NEGATIVE_Z);
					// Com_Printf("\n\n");

					// PITCH, YAW, ROLL
					float humerus_angles_bis[3] = {humerus_angles[PITCH], humerus_angles[YAW], humerus_angles[ROLL]};

					// 5 3 4
					// G2_Set_Bone_Angles(&ghoul2[model_ind], ghoul2[model_ind].mBlist, "rhumerus", humerus_angles_bis, BONE_ANGLES_REPLACE, NEGATIVE_Z, POSITIVE_Y, NEGATIVE_X, 0, 0);

					

					// get coordinates of all bolts
					std::vector<float[3]> bolt_pos(ghoul2[model_ind].mBltlist.size());
					for(int bolt_ind = 0; bolt_ind < ghoul2[model_ind].mBltlist.size(); bolt_ind ++) {

						// std::cout << "bolt index: " << bolt_ind << ", associated bone ind: " << ghoul2[model_ind].mBltlist[bolt_ind].boneNumber << std::endl; 

						// model_ind, bolt_ind
						G2_GetBoltMatrixLow(ghoul2[model_ind], bolt_ind, scale, retMatrix);
						// std::cout << std::endl << std::endl << std::endl;
						// std::cout << "called! size= " << ghoul2.size() << std::endl;

						
						G2_GenerateWorldMatrix(backEnd.refdef.entities[entity_ind].e.angles, backEnd.refdef.entities[entity_ind].e.origin);
						Multiply_3x4Matrix(&matrix, &local_entity_matrix, &retMatrix);

						// Com_Printf("Entity angles: %.3f %.3f %.3f\n", backEnd.refdef.entities[entity_ind].e.angles[0], backEnd.refdef.entities[entity_ind].e.angles[1], backEnd.refdef.entities[entity_ind].e.angles[2]);

						p1[0] = matrix.matrix[0][3];
						p1[1] = matrix.matrix[1][3];
						p1[2] = matrix.matrix[2][3];

						// record bolt positions
						bolt_pos[bolt_ind][0] = matrix.matrix[0][3];
						bolt_pos[bolt_ind][1] = matrix.matrix[1][3];
						bolt_pos[bolt_ind][2] = matrix.matrix[2][3];
						
						p2[0] = p1[0] + 100;
						p2[1] = p1[1] + 100;
						p2[2] = p1[2] + 100;

						// draw 3d local axis
						// qglBegin (GL_LINES);
						// qglColor3f (1,1,1);
						// qglVertex3fv (bolt_pos[bolt_ind]);
						// qglVertex3fv (p2);
						// qglEnd();

						// // float matrix[3][4];
						// for(int i = 0; i < 3; i ++) {
						// 	for(int j = 0; j < 4; j ++) {
						// 		std::cout << retMatrix.matrix[i][j]  <<" "; 
						// 	}
						// 	std::cout << endl;
						// }
						// std::cout << std::endl << std::endl << std::endl;
					}

					// std::cout << "n bones alt: " << ghoul2[model_ind].mBoneCache->mNumBones << std::endl;
					std::map<std::string, size_t> bone_name_ind; // retreive bone index from name 
					std::vector<std::vector<float>> bones_pos; 
					std::vector<mdxaBone_t> bones_matrix; // store the matrix of the bones
					if(ghoul2[model_ind].mBoneCache != 0) {

						// draw an identity matrix
						std::vector<float> custom_pos = {backEnd.refdef.entities[entity_ind].e.origin[0], backEnd.refdef.entities[entity_ind].e.origin[1]-20, backEnd.refdef.entities[entity_ind].e.origin[2]+50};
						drawMatrix(custom_pos, identityMatrix);
						
						// drawMatrix(custom_pos, local_entity_matrix);

						
						//------------------

						// std::cout << "n bones alt: " << ghoul2[model_ind].mBoneCache->mNumBones << std::endl;
						bones_pos.resize(ghoul2[model_ind].mBoneCache->mNumBones, std::vector<float>(3, 0));
						bones_matrix.resize(bones_pos.size());

						// record bone positions
						for(size_t bone_ind = 0; bone_ind <  ghoul2[model_ind].mBoneCache->mNumBones; bone_ind ++) {
							mdxaSkel_t			*skel;
							mdxaSkelOffsets_t	*offsets;
   							offsets = (mdxaSkelOffsets_t *)((byte *)ghoul2[model_ind].aHeader + sizeof(mdxaHeader_t));
							skel = (mdxaSkel_t *)((byte *)ghoul2[model_ind].aHeader + sizeof(mdxaHeader_t) + offsets->offsets[bone_ind]);
							// std::cout << skel->name << std::endl;

							mdxaBone_t test_bolt, ret_matrix;
							Multiply_3x4Matrix(&test_bolt, (mdxaBone_t *)&ghoul2[model_ind].mBoneCache->Eval(bone_ind), &skel->BasePoseMat); // DEST FIRST ARG
							Multiply_3x4Matrix(&ret_matrix, &local_entity_matrix, &test_bolt);

							copy_bone_matrix(bones_matrix[bone_ind], ret_matrix); // store the matrix to be used later

							// -> do not transform by current animation
							// Multiply_3x4Matrix(&ret_matrix, &worldMatrix, &skel->BasePoseMat);

							bones_pos[bone_ind][0] = ret_matrix.matrix[0][3];
							bones_pos[bone_ind][1] = ret_matrix.matrix[1][3];
							bones_pos[bone_ind][2] = ret_matrix.matrix[2][3];
							bone_name_ind[skel->name] = bone_ind;

						}

						// draw bones matrix
						for(size_t bone_ind = 0; bone_ind <  ghoul2[model_ind].mBoneCache->mNumBones; bone_ind ++) {
							mdxaSkel_t			*skel;
							mdxaSkelOffsets_t	*offsets;
   							offsets = (mdxaSkelOffsets_t *)((byte *)ghoul2[model_ind].aHeader + sizeof(mdxaHeader_t));
							skel = (mdxaSkel_t *)((byte *)ghoul2[model_ind].aHeader + sizeof(mdxaHeader_t) + offsets->offsets[bone_ind]);
							// std::cout << skel->name << std::endl;

							bone_name_ind[skel->name] = bone_ind;

							// debug display local matrix
							if(std::string(skel->name) == std::string("rhumerus")) {
								mdxaBone_t* temp = (mdxaBone_t *)&ghoul2[model_ind].mBoneCache->Eval(bone_ind);
								mdxaBone_t temp2;
								Multiply_3x4Matrix(&temp2, temp, &skel->BasePoseMat); // DEST FIRST ARG

								Com_Printf("local transformed bone humerus matrix:\n");
								Com_Printf("%.3f %.3f %.3f\n", temp2.matrix[0][0], temp2.matrix[0][1], temp2.matrix[0][2]);
								Com_Printf("%.3f %.3f %.3f\n", temp2.matrix[1][0], temp2.matrix[1][1], temp2.matrix[1][2]);
								Com_Printf("%.3f %.3f %.3f\n", temp2.matrix[2][0], temp2.matrix[2][1], temp2.matrix[2][2]);
								Com_Printf("\n\n");

								Com_Printf("basePoseMat:\n");
								Com_Printf("%.3f %.3f %.3f\n", skel->BasePoseMat.matrix[0][0], skel->BasePoseMat.matrix[0][1], skel->BasePoseMat.matrix[0][2]);
								Com_Printf("%.3f %.3f %.3f\n", skel->BasePoseMat.matrix[1][0], skel->BasePoseMat.matrix[1][1], skel->BasePoseMat.matrix[1][2]);
								Com_Printf("%.3f %.3f %.3f\n", skel->BasePoseMat.matrix[2][0], skel->BasePoseMat.matrix[2][1], skel->BasePoseMat.matrix[2][2]);
								Com_Printf("\n\n");

								Com_Printf("basePoseInv:\n");
								Com_Printf("%.3f %.3f %.3f\n", skel->BasePoseMatInv.matrix[0][0], skel->BasePoseMatInv.matrix[0][1], skel->BasePoseMatInv.matrix[0][2]);
								Com_Printf("%.3f %.3f %.3f\n", skel->BasePoseMatInv.matrix[1][0], skel->BasePoseMatInv.matrix[1][1], skel->BasePoseMatInv.matrix[1][2]);
								Com_Printf("%.3f %.3f %.3f\n", skel->BasePoseMatInv.matrix[2][0], skel->BasePoseMatInv.matrix[2][1], skel->BasePoseMatInv.matrix[2][2]);
								Com_Printf("\n\n");
							}

							// compute bone position ?
							mdxaBone_t test_bolt, ret_matrix;

							Multiply_3x4Matrix(&test_bolt, (mdxaBone_t *)&ghoul2[model_ind].mBoneCache->Eval(bone_ind), &skel->BasePoseMat); // DEST FIRST ARG
							Multiply_3x4Matrix(&ret_matrix, &local_entity_matrix, &test_bolt);

							if(std::find(keep_bones.begin(), keep_bones.end(), std::string(skel->name)) == keep_bones.end()) {
								continue;	
							}

							if(std::find(keep_axis.begin(), keep_axis.end(), std::string(skel->name)) == keep_axis.end()) {
								continue;
							}

							// draw the local bone matrix
							drawMatrix(bones_pos[bone_ind], ret_matrix);

							// draw shifted bone matrix for one specific bone
							if(std::string(skel->name) == std::string("rhumerus")) {
								
								// draw transformed matrix
								std::vector<float> custom_pos = {backEnd.refdef.entities[entity_ind].e.origin[0], backEnd.refdef.entities[entity_ind].e.origin[1]-30, backEnd.refdef.entities[entity_ind].e.origin[2]+50};
								{mdxaBone_t temp;
								// undo entity view axis
								drawMatrix(custom_pos, ret_matrix);

								}

								// draw original matrix
								custom_pos = {backEnd.refdef.entities[entity_ind].e.origin[0], backEnd.refdef.entities[entity_ind].e.origin[1]+30, backEnd.refdef.entities[entity_ind].e.origin[2]+50};
								// multiply by the entity matrix
								{mdxaBone_t temp;
								
								Com_Printf("eigen generated matrix:\n");
								Com_Printf("%.3f %.3f %.3f\n", test_mat.matrix[0][0], test_mat.matrix[0][1], test_mat.matrix[0][2]);
								Com_Printf("%.3f %.3f %.3f\n", test_mat.matrix[1][0], test_mat.matrix[1][1], test_mat.matrix[1][2]);
								Com_Printf("%.3f %.3f %.3f\n", test_mat.matrix[2][0], test_mat.matrix[2][1], test_mat.matrix[2][2]);
								Com_Printf("\n\n");
								
								Multiply_3x4Matrix(&temp, &local_entity_matrix, &test_mat);
								drawMatrix(custom_pos, temp);}

								

								// compute and draw a matrix from the angles computed by ja engine
								// AngleVectors( const vec3_t angles, vec3_t forward, vec3_t right, vec3_t up)
								custom_pos = {backEnd.refdef.entities[entity_ind].e.origin[0], backEnd.refdef.entities[entity_ind].e.origin[1]+30, backEnd.refdef.entities[entity_ind].e.origin[2]+70};
								
								// vec3_t angles = {angle0, angle1, angle2}; // 0 -> PITCH ; 1 -> YAW ; 2 -> ROLL
								vec3_t angles = {humerus_angles[PITCH], humerus_angles[YAW], humerus_angles[ROLL]}; // angles in degrees

								
								// vec3_t forward, right, up;
								// AngleVectors(angles, forward, right, up);
								{mdxaBone_t test2, temp, temp2;
								// test_mat.matrix[0][0] = forward[0]; test_mat.matrix[0][1] = right[0]; test_mat.matrix[0][2] = up[0]; 
								// test_mat.matrix[1][0] = forward[1]; test_mat.matrix[1][1] = right[1]; test_mat.matrix[1][2] = up[1]; 
								// test_mat.matrix[2][0] = forward[2]; test_mat.matrix[2][1] = right[2]; test_mat.matrix[2][2] = up[2]; 

								Create_Matrix(angles, &test2); // angles to matrix

								Com_Printf("ja generated matrix:\n");
								Com_Printf("%.3f %.3f %.3f\n", test2.matrix[0][0], test2.matrix[0][1], test2.matrix[0][2]);
								Com_Printf("%.3f %.3f %.3f\n", test2.matrix[1][0], test2.matrix[1][1], test2.matrix[1][2]);
								Com_Printf("%.3f %.3f %.3f\n", test2.matrix[2][0], test2.matrix[2][1], test2.matrix[2][2]);
								Com_Printf("\n\n");

								// Multiply_3x4Matrix(&temp, &test2, &skel->BasePoseMat);

								Multiply_3x4Matrix(&temp2, &local_entity_matrix, &test2);

								// temp2.matrix[0][0] = 1; temp2.matrix[0][1] = 0; temp2.matrix[0][2] = 0;
								// temp2.matrix[1][0] = 0; temp2.matrix[1][1] = -1; temp2.matrix[1][2] = 0;
								// temp2.matrix[2][0] = 0; temp2.matrix[2][1] = 0; temp2.matrix[2][2] = 1;
								// Multiply_3x4Matrix(&temp3, &temp, &temp2);
								// drawMatrix(custom_pos, temp2);
								}

								


								// Com_Printf("rhumerus bone matrix:\n");
								// Com_Printf("%.3f %.3f %.3f\n", ret_matrix.matrix[0][0], ret_matrix.matrix[0][1], ret_matrix.matrix[0][2]);
								// Com_Printf("%.3f %.3f %.3f\n", ret_matrix.matrix[1][0], ret_matrix.matrix[1][1], ret_matrix.matrix[1][2]);
								// Com_Printf("%.3f %.3f %.3f\n", ret_matrix.matrix[2][0], ret_matrix.matrix[2][1], ret_matrix.matrix[2][2]);
								// Com_Printf("\n\n");

								// compute and print humerus matrix relative to players orientation
								mdxaBone_t player_inv, humerus_local;
								Inverse_Matrix(&local_entity_matrix, &player_inv);
								Multiply_3x4Matrix(&humerus_local, &ret_matrix, &player_inv);

								// Com_Printf("rhumerus local matrix:\n");
								// Com_Printf("%.3f %.3f %.3f\n", humerus_local.matrix[0][0], humerus_local.matrix[0][1], humerus_local.matrix[0][2]);
								// Com_Printf("%.3f %.3f %.3f\n", humerus_local.matrix[1][0], humerus_local.matrix[1][1], humerus_local.matrix[1][2]);
								// Com_Printf("%.3f %.3f %.3f\n", humerus_local.matrix[2][0], humerus_local.matrix[2][1], humerus_local.matrix[2][2]);
								// Com_Printf("\n\n");


								// draw humerus multiply by angle extraction effect
								mdxaBone_t temp, test, mult;

								// 1 0 0
								// 0 1 0
								// 0 0 1

								// ->

								//   0  0 -1
								//   0  1 0
								//  -1  0 0

								// inv 

								// 0   0 -1
								// 0   1 0
								// -1  0 0

								// test.matrix[0][0] = 0; test.matrix[0][1] = 0; test.matrix[0][2] = -1;
								// test.matrix[1][0] = 0; test.matrix[1][1] = 1; test.matrix[1][2] = 0;
								// test.matrix[2][0] = -1; test.matrix[2][1] = 0; test.matrix[2][2] = 0;

								// test.matrix[0][0] = 1; test.matrix[0][1] = 0; test.matrix[0][2] = 0;
								// test.matrix[1][0] = 0; test.matrix[1][1] = 1; test.matrix[1][2] = 0;
								// test.matrix[2][0] = 0; test.matrix[2][1] = 0; test.matrix[2][2] = 1;

								// copy_bone_matrix(temp, ret_matrix);
								// Multiply_3x4Matrix(&mult, &temp, &test);
								// custom_pos = {backEnd.refdef.entities[entity_ind].e.origin[0], backEnd.refdef.entities[entity_ind].e.origin[1]-30, backEnd.refdef.entities[entity_ind].e.origin[2]+70};
								// drawMatrix(custom_pos, mult);



							}

							// for the radius, try to cancel the parent (humerus) rotation
							if(std::string(skel->name) == std::string("rradius")) {
								std::vector<float> custom_pos;
								
								// try to multipy by the inverse of the humerus bone current matrix
								// to cancel its transformation
								
								mdxaBone_t radius_copy;
								copy_bone_matrix(radius_copy, bones_matrix[bone_name_ind["rradius"]]);

								mdxaBone_t rhumerus_temp, ruhmerus_inv;
								copy_bone_matrix(rhumerus_temp, bones_matrix[bone_name_ind["rhumerus"]]);

								
								// void Inverse_Matrix(mdxaBone_t *src, mdxaBone_t *dest);
								// src -> dest
								Inverse_Matrix(&rhumerus_temp, &ruhmerus_inv);
								for(int i = 0; i < 3; i ++) {
									ruhmerus_inv.matrix[i][3] = 0;
								}

								// draw radius multiplied by humerus inverse
								mdxaBone_t radius_out;
								Multiply_3x4Matrix(&radius_out, &ruhmerus_inv, &radius_copy);

								custom_pos = {backEnd.refdef.entities[entity_ind].e.origin[0], backEnd.refdef.entities[entity_ind].e.origin[1]-30, backEnd.refdef.entities[entity_ind].e.origin[2]+30};
								// drawMatrix(custom_pos, radius_out);

								// draw radius normal matrix
								custom_pos = {backEnd.refdef.entities[entity_ind].e.origin[0], backEnd.refdef.entities[entity_ind].e.origin[1]+30, backEnd.refdef.entities[entity_ind].e.origin[2]+30};
								// drawMatrix(custom_pos, radius_copy);

								// Compute euler angles with eigen
								Eigen::Matrix3f m;
								for(int i = 0; i < 3; i ++) {
									for(int j = 0; j < 3; j ++) {
										m(i,j) = ruhmerus_inv.matrix[i][j];
									}
								}

								// Com_Printf("Inverse rhumerus matrix:\n");
								// for(int i = 0; i < 3; i ++) {
								// 	Com_Printf("%.3f %.3f %.3f\n", ruhmerus_inv.matrix[i][0], ruhmerus_inv.matrix[i][1], ruhmerus_inv.matrix[i][2]);
								// }
								// Com_Printf("\n");

								// m(0,0) = 0; m(0,1) = 0; m(0,2) = 1;
								// m(1,0) = 0; m(1,1) = -1; m(1,2) = 0;
								// m(2,0) = -1; m(2,1) = 0; m(2,2) = 0;

								// Eigen::Matrix<float,3,1> res = m.eulerAngles(2,1,0);
								// Eigen::Matrix<float,3,1> res = m.canonicalEulerAngles(2,1,0);

								// float test_angles[3];
								// test_angles[ROLL] = res(0)*180./M_PI;
								// test_angles[PITCH] = res(1)*180./M_PI;
								// test_angles[YAW] = res(2)*180./M_PI;

								// / angle indexes
								// #define	PITCH	0		// up / down
								// #define	YAW		1		// left / right
								// #define	ROLL	2		// fall over

								// debug from https://stackoverflow.com/questions/11514063/extract-yaw-pitch-and-roll-from-a-rotationmatrix

								// G2_Set_Bone_Angles(&ghoul2[model_ind], ghoul2[model_ind].mBlist, "rhumerus", test_angles, BONE_ANGLES_REPLACE, POSITIVE_X, POSITIVE_Y, POSITIVE_Z, 0, 0);
								
								// G2_Set_Bone_Angles(CGhoul2Info *ghlInfo, boneInfo_v &blist, const char *boneName, const float *angles, const int flags, const Eorientations up, const Eorientations left, const Eorientations forward, const int blendTime, const int currentTime);

								// in this call with PREMULT, angles are relatives to the parent bone
								// G2_Set_Bone_Angles(&ghoul2[model_ind], ghoul2[model_ind].mBlist, "rradius", test_angles, BONE_ANGLES_PREMULT, POSITIVE_X, POSITIVE_Y, POSITIVE_Z, 0, 0);

								// in this call with REPLACE, angles are relatives to the entitiy matrix
								// vec3_t debug_angles = {90, 0, 45};
								// G2_Set_Bone_Angles(&ghoul2[model_ind], ghoul2[model_ind].mBlist, "rradius", debug_angles, BONE_ANGLES_REPLACE, POSITIVE_X, POSITIVE_Y, POSITIVE_Z, 0, 0);

							}

						}



						for(size_t bone_ind = 0; bone_ind <  ghoul2[model_ind].mBoneCache->mNumBones; bone_ind ++) {
							mdxaSkel_t			*skel;
							mdxaSkelOffsets_t	*offsets;
   							offsets = (mdxaSkelOffsets_t *)((byte *)ghoul2[model_ind].aHeader + sizeof(mdxaHeader_t));
							skel = (mdxaSkel_t *)((byte *)ghoul2[model_ind].aHeader + sizeof(mdxaHeader_t) + offsets->offsets[bone_ind]);

							for(size_t ch_ind = 0; ch_ind < skel->numChildren; ch_ind ++) {
								int child_ind = skel->children[ch_ind];
								mdxaSkel_t * skel_child = (mdxaSkel_t *)((byte *)ghoul2[model_ind].aHeader + sizeof(mdxaHeader_t) + offsets->offsets[child_ind]);

								if(std::find(keep_bones.begin(), keep_bones.end(), std::string(skel_child->name)) == keep_bones.end()) {
									continue;	
								}

								for(int j = 0; j < 3; j ++) {
									p1[j] = bones_pos[bone_ind][j];
									p2[j] = bones_pos[child_ind][j];
								}
								qglBegin (GL_LINES);
								qglColor3f (1,1,1);
								qglVertex3fv (p1);
								qglVertex3fv (p2);
								qglEnd();
							}
						}
					
					}

					// std::cout << "nb bones for this model: " << ghoul2[model_ind].mBlist.size() << std::endl;
					for(int bone_ind = 0; bone_ind < ghoul2[model_ind].mBlist.size(); bone_ind ++) {
					
						// void G2_GetBoneMatrixLow(CGhoul2Info &ghoul2,int boneNum,const vec3_t scale,mdxaBone_t &retMatrix,mdxaBone_t *&retBasepose,mdxaBone_t *&retBaseposeInv);
						// int G2_GetParentBoneMatrixLow(CGhoul2Info &ghoul2,int boneNum,const vec3_t scale,mdxaBone_t &retMatrix,mdxaBone_t *&retBasepose,mdxaBone_t *&retBaseposeInv);

						vec3_t scale = {1., 1., 1.};
						// G2_GetBoneMatrixLow(ghoul2[model_ind], bone_ind, scale, ghoul2[model_ind].mBlist[bone_ind].originalTrueBoneMatrix, ghoul2[model_ind].mBlist[bone_ind].basepose, ghoul2[model_ind].mBlist[bone_ind].baseposeInv);
						// G2_GetBoneBasepose(ghoul2[model_ind],bone_ind, ghoul2[model_ind].mBlist[bone_ind].basepose, ghoul2[model_ind].mBlist[bone_ind].baseposeInv);
						// mdxaBone_t& basePos =  ghoul2[model_ind].mBlist[bone_ind].newMatrix;

						// get the parent bone index and then get the two associated bolts

						// G2_GetParentBoneMatrixLow(ghoul2[model_ind], bone_ind, scale, ghoul2[model_ind].mBlist[bone_ind].parentOriginalTrueBoneMatrix, ghoul2[model_ind].mBlist[bone_ind].baseposeParent, ghoul2[model_ind].mBlist[bone_ind].baseposeInvParent);
						// mdxaBone_t basePosParent = *ghoul2[model_ind].mBlist[bone_ind].baseposeParent;


						// std::cout << endl << endl;
						// std::cout << "Current bone number: " << ghoul2[model_ind].mBlist[bone_ind].boneNumber << std::endl;
						// std::cout << "matrix: " << std::endl;
						// for(int i = 0; i < 3; i ++) {
						// 	for(int j = 0; j < 4; j ++) {
						// 		std::cout << ghoul2[model_ind].mBlist[bone_ind].matrix.matrix[i][j]  <<" "; 
						// 	}
						// 	std::cout << endl;
						// }
						// std::cout << std::endl;
						// for(int i = 0; i < 3; i ++) {
						// 	for(int j = 0; j < 4; j ++) {
						// 		std::cout << ghoul2[model_ind].mBlist[bone_ind].newMatrix.matrix[i][j]  <<" "; 
						// 	}
						// 	std::cout << endl;
						// }
						// std::cout << "original origin: " << std::endl;
						// std::cout << ghoul2[model_ind].mBlist[bone_ind].originalOrigin[0] << ", " << ghoul2[model_ind].mBlist[bone_ind].originalOrigin[1] <<  " " << ghoul2[model_ind].mBlist[bone_ind].originalOrigin[2] << std::endl;

						// std::cout << "last position: " << std::endl;
						// std::cout << ghoul2[model_ind].mBlist[bone_ind].lastPosition[0] << ", " << ghoul2[model_ind].mBlist[bone_ind].lastPosition[1] <<  " " << ghoul2[model_ind].mBlist[bone_ind].lastPosition[2] << std::endl;


						int bolt_ind = G2_Find_Bolt_Bone_Num(ghoul2[model_ind].mBltlist, ghoul2[model_ind].mBlist[bone_ind].boneNumber);
						// std::cout << "Bolt ind for this bone: " << bolt_ind << std::endl;
						// std::cout << "verif bone ind from bolt ind: " << ghoul2[model_ind].mBltlist[bolt_ind].boneNumber << std::endl;


						// std::cout << "Bone index compare: " << bone_ind << ", " << ghoul2[model_ind].mBlist[bone_ind].boneNumber << std::endl;

						// std::cout << "Parent Bone index compare: " << ghoul2[model_ind].mBlist[bone_ind].parentBoneIndex;
						// std::cout << ", " << ghoul2[model_ind].mBlist[ghoul2[model_ind].mBlist[bone_ind].parentBoneIndex].boneNumber << std::endl;

						float test2[3];
						test2[0] = bolt_pos[bolt_ind][0]+10;
						test2[1] = bolt_pos[bolt_ind][1]+10;
						test2[2] = bolt_pos[bolt_ind][2]+10;

						// qglBegin (GL_LINES);
						// qglColor3f (0,0,1);
						// qglVertex3fv (bolt_pos[bolt_ind]);
						// qglVertex3fv (test2);
						// qglEnd();

						// mdxaSkel_t
						// mdxaSkel_t			*skel;
						// mdxaSkelOffsets_t	*offsets;
   						// offsets = (mdxaSkelOffsets_t *)((byte *)ghlInfo->aHeader + sizeof(mdxaHeader_t));
						// skel = (mdxaSkel_t *)((byte *)ghlInfo->aHeader + sizeof(mdxaHeader_t) + offsets->offsets[0]);
						// From int G2_Find_Bone(CGhoul2Info *ghlInfo, boneInfo_v &blist, const char *boneName)

						mdxaSkel_t			*skel;
						mdxaSkelOffsets_t	*offsets;
   						offsets = (mdxaSkelOffsets_t *)((byte *)ghoul2[model_ind].aHeader + sizeof(mdxaHeader_t));
						skel = (mdxaSkel_t *)((byte *)ghoul2[model_ind].aHeader + sizeof(mdxaHeader_t) + offsets->offsets[ghoul2[model_ind].mBlist[bone_ind].boneNumber]);
						// std::cout << "skel name: " << skel->name << std::endl;
						// std::cout << "num children: " << skel->numChildren << std::endl;
						// std::cout << std::endl << std::endl;
						for(int i = 0; i < skel->numChildren; i ++) {

							mdxaSkel_t* skelbis = (mdxaSkel_t *)((byte *)ghoul2[model_ind].aHeader + sizeof(mdxaHeader_t) + offsets->offsets[skel->children[i]]);
							
							// std::cout << "children " << i << " -> " << skel->children[i] << " "; 
							// std::cout << skelbis->name << std::endl;

							mdxaBone_t test_bolt, ret_matrix;
							Multiply_3x4Matrix(&test_bolt, (mdxaBone_t *)&ghoul2[model_ind].mBoneCache->Eval(skel->children[i]), &skelbis->BasePoseMat); // DEST FIRST ARG
							Multiply_3x4Matrix(&ret_matrix, &worldMatrix, &test_bolt);
							for(int j = 0; j < 3; j ++) {
								p1[j] = ret_matrix.matrix[j][3];
								p2[j] = p1[j] + 10;
							}
							// qglBegin (GL_LINES);
							// qglColor3f (0,1,0);
							// qglVertex3fv (p1);
							// qglVertex3fv (p2);
							// qglEnd();

							// for(int j = 0; j < 3; j ++) {
							// 	for(int k = 0; k < 4; k ++) {
							// 		std::cout << skelbis->BasePoseMat.matrix[j][k] << " ";
							// 	}
							// 	std::cout << std::endl;
							// }
							// std::cout << std::endl;
							
							// for(int j = 0; j < 3; j ++) {
							// 	for(int k = 0; k < 4; k ++) {
							// 		std::cout << ghoul2[model_ind].mBoneCache->mFinalBones[skel->children[i]].boneMatrix.matrix[j][k] << " ";
							// 	}
							// 	std::cout << std::endl;
							// }
							// std::cout << std::endl;

							int bolt_ind = G2_Find_Bolt_Bone_Num(ghoul2[model_ind].mBltlist, skel->children[i]);

							// try to display the bone
							for(int j = 0; j < 3; j ++) {
								p1[j] = ghoul2[model_ind].mBoneCache->mFinalBones[skel->children[i]].boneMatrix.matrix[j][3];
								p1[j] += backEnd.refdef.entities[entity_ind].e.origin[j];
								p1[j] += bolt_pos[bolt_ind][j];
								p2[j] = p1[j]+10;
							}
							// qglBegin (GL_LINES);
							// qglColor3f (0,1,0);
							// qglVertex3fv (p1);
							// qglVertex3fv (p2);
							// qglEnd();


						}
						int bone_number_from_skel = G2_Find_Bone(&ghoul2[model_ind], ghoul2[model_ind].mBlist, skel->name);
						// std::cout << "bone index from skeleton name: " << bone_number_from_skel << " vs " << bone_ind << " and " << ghoul2[model_ind].mBlist[bone_ind].boneNumber << std::endl;

						// int parent_bone_ind = G2_Find_Bone_In_List(ghoul2[model_ind].mBlist, ghoul2[model_ind].mBlist[bone_ind].parentBoneIndex);
						// std::cout << "parent bone index alternative: " << parent_bone_ind << std::endl;

						mdxaSkel_t	*skel_parent = (mdxaSkel_t *)((byte *)ghoul2[model_ind].aHeader + sizeof(mdxaHeader_t) + offsets->offsets[skel->parent]);
						// std::cout << "parent bone name: " << skel_parent->name << std::endl;

						// std::cout << "attempt get list of bones" << std::endl;
						// std::cout << ghoul2[model_ind].mBoneCache->header->numBones << std::endl;
						// std::cout << std::endl;

						// if(ghoul2[model_ind].mBlist[bone_ind].parentBoneIndex >= 0) {
						// 	int parent_bolt_ind = G2_Find_Bolt_Bone_Num(ghoul2[model_ind].mBltlist, ghoul2[model_ind].mBlist[bone_ind].parentBoneIndex);
						// 	std::cout << "Parent Bolt ind for this bone: " << parent_bolt_ind << std::endl;

						// }

						// attempt bone position with the right index
						// if(ghoul2[model_ind].mBlist[bone_ind].boneNumber >= 0) {
						// 	G2_GetBoneMatrixLow(ghoul2[model_ind], ghoul2[model_ind].mBlist[bone_ind].boneNumber, scale, ghoul2[model_ind].mBlist[bone_ind].originalTrueBoneMatrix, ghoul2[model_ind].mBlist[bone_ind].basepose , ghoul2[model_ind].mBlist[bone_ind].baseposeInv);
						// 	std::cout << "new attempt bone position: " << std::endl;
						// 	for(int i = 0; i < 3; i ++) {
						// 		for(int j = 0; j < 4; j ++) {
						// 			std::cout << ghoul2[model_ind].mBlist[bone_ind].originalTrueBoneMatrix.matrix[i][j] << " ";
						// 		}
						// 		std::cout << std::endl;
						// 	}
						// 	std::cout << std::endl;
						// 	p1[0] = ghoul2[model_ind].mBlist[bone_ind].originalTrueBoneMatrix.matrix[0][3];
						// 	p1[1] = ghoul2[model_ind].mBlist[bone_ind].originalTrueBoneMatrix.matrix[1][3];
						// 	p1[2] = ghoul2[model_ind].mBlist[bone_ind].originalTrueBoneMatrix.matrix[2][3];

						// 	int bParentListIndex = G2_Find_Bone(&ghoul2[model_ind], ghoul2[model_ind].mBlist, skel_parent->name);

						// 	if(bParentListIndex > 0) {
								
						// 		// int parent_bolt_ind = G2_Find_Bolt_Bone_Num(ghoul2[model_ind].mBltlist, ghoul2[model_ind].mBlist[bParentListIndex].boneNumber);
						// 		// std::cout << "Parent Bolt ind for this bone: " << parent_bolt_ind << std::endl;
								
						// 		G2_GetBoneMatrixLow(ghoul2[model_ind], ghoul2[model_ind].mBlist[bParentListIndex].boneNumber, scale, ghoul2[model_ind].mBlist[bParentListIndex].originalTrueBoneMatrix, ghoul2[model_ind].mBlist[bParentListIndex].basepose , ghoul2[model_ind].mBlist[bParentListIndex].baseposeInv);
						// 		p2[0] = ghoul2[model_ind].mBlist[bParentListIndex].originalTrueBoneMatrix.matrix[0][3];
						// 		p2[1] = ghoul2[model_ind].mBlist[bParentListIndex].originalTrueBoneMatrix.matrix[1][3];
						// 		p2[2] = ghoul2[model_ind].mBlist[bParentListIndex].originalTrueBoneMatrix.matrix[2][3];
								
						// 		std::cout << "p1 test: " << p1[0] << ", " << p1[1] << ", " << p1[2] << std::endl;
						// 		std::cout << "p2 test: " << p2[0] << ", " << p2[1] << ", " << p2[2] << std::endl;
						// 	}

						// }

						// G2_GenerateWorldMatrix(backEnd.refdef.entities[entity_ind].e.angles, backEnd.refdef.entities[entity_ind].e.origin);
						
						// Multiply_3x4Matrix(&matrix, &worldMatrix, &basePos);
						// p1[0] = matrix.matrix[0][3];
						// p1[1] = matrix.matrix[1][3];
						// p1[2] = matrix.matrix[2][3];
						// p1[0] = ghoul2[model_ind].mBlist[bone_ind].lastPosition[0]+backEnd.refdef.entities[entity_ind].e.origin[0];
						// p1[1] = ghoul2[model_ind].mBlist[bone_ind].lastPosition[1]+backEnd.refdef.entities[entity_ind].e.origin[1];
						// p1[2] = ghoul2[model_ind].mBlist[bone_ind].lastPosition[2]+backEnd.refdef.entities[entity_ind].e.origin[2];

						// p2[0] = p1[0]+10;
						// p2[1] = p1[1]+10;
						// p2[2] = p1[2]+10;

						// qglBegin (GL_LINES);
						// qglColor3f (1,0,0);
						// qglVertex3fv (p1);
						// qglVertex3fv (p2);
						// qglEnd();

						// std::cout << "Parent bone: " << std::endl;
						// for(int i = 0; i < 3; i ++) {
						// 	for(int j = 0; j < 4; j ++) {
						// 		std::cout << baseParentPos->matrix[i][j]  <<" "; 
						// 	}
						// 	std::cout << endl;
						// }
						// std::cout << endl << endl;


					}


				}



			}

		}
		

		// qglTranslatef(-backEnd.refdef.entities[entity_ind].e.origin[0], -backEnd.refdef.entities[entity_ind].e.origin[1], -backEnd.refdef.entities[entity_ind].e.origin[2]);


		bbox_ind ++;
		// if(bbox_ind > 10) {
		// 	break;
		// }
	}

		qglDepthRange( 0, 1 );

}