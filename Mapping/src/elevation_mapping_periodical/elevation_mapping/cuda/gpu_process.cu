/*
 * gpu_process.cu
 *
 *  Created on: Aug 1, 2019
 *      Author: YiYuan PAN, Peter XU
 *	 Institute: ZJU, Robotics 104
 */

#include <cuda_runtime.h>
#include <stdio.h>
#include <iostream>
#include<cmath>
#include<iomanip>
// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>


using namespace std;
__device__ float *map_lowest;
__device__ float *map_elevation;
__device__ float *map_variance;
__device__ float *map_intensity;
__device__ float *map_traver;

__device__ int *map_colorR;
__device__ int *map_colorG;
__device__ int *map_colorB;

__device__ float central_coordinate[2];
__device__ int start_indice[2];

__device__ float sensorZatLowestScan;

__constant__ int Length;
__constant__ float Resolution;
__constant__ float obstacle_threshold;

//points_
__constant__ int C_point_num;
__constant__ float C_min_r;
__constant__ float C_beam_a;
__constant__ float C_beam_c;
__constant__ float C_factor_a;
__constant__ float C_factor_b;
__constant__ float C_factor_c;
__constant__ float C_factor_d;
__constant__ float C_factor_e;
__constant__ float C_lateral_factor;

//remove point
__constant__ double C_relativeLowerThreshold;
__constant__ double C_relativeUpperThreshold;

//fuse
__constant__ float mahalanobisDistanceThreshold_;

struct Pos3
{
    float x;
    float y;
    float z;
};


__device__ void computerEigenvalue(float *pMatrix,int nDim, float *maxvector, float dbEps,int nJt)
{
    float pdblVects[9];
    float pdbEigenValues[3];
    
	for(int i = 0; i < nDim; i ++) 
	{   
		pdblVects[i*nDim+i] = 1.0f; 
		for(int j = 0; j < nDim; j ++) 
		{ 
			if(i != j)   
				pdblVects[i*nDim+j]=0.0f; 
		} 
	} 
 
	int nCount = 0;	//迭代次数
	while(1)
	{
		//在pMatrix的非对角线上找到最大元素
		float dbMax = pMatrix[1];
		int nRow = 0;
		int nCol = 1;
		for (int i = 0; i < nDim; i ++)	//行
		{
			for (int j = 0; j < nDim; j ++)	//列
			{
				float d = fabs(pMatrix[i*nDim+j]); 
 
				if((i!=j) && (d> dbMax)) 
				{ 
					dbMax = d;   
					nRow = i;   
					nCol = j; 
				} 
			}
		}
 
		if(dbMax < dbEps) //精度符合要求 
			break;  
 
		if(nCount > nJt) //迭代次数超过限制
			break;
 
		nCount++;
 
		float dbApp = pMatrix[nRow*nDim+nRow];
		float dbApq = pMatrix[nRow*nDim+nCol];
		float dbAqq = pMatrix[nCol*nDim+nCol];
 
		//计算旋转角度
		float dbAngle = 0.5*atan2(-2*dbApq,dbAqq-dbApp);
		float dbSinTheta = sin(dbAngle);
		float dbCosTheta = cos(dbAngle);
		float dbSin2Theta = sin(2*dbAngle);
		float dbCos2Theta = cos(2*dbAngle);
 
		pMatrix[nRow*nDim+nRow] = dbApp*dbCosTheta*dbCosTheta + 
			dbAqq*dbSinTheta*dbSinTheta + 2*dbApq*dbCosTheta*dbSinTheta;
		pMatrix[nCol*nDim+nCol] = dbApp*dbSinTheta*dbSinTheta + 
			dbAqq*dbCosTheta*dbCosTheta - 2*dbApq*dbCosTheta*dbSinTheta;
		pMatrix[nRow*nDim+nCol] = 0.5*(dbAqq-dbApp)*dbSin2Theta + dbApq*dbCos2Theta;
		pMatrix[nCol*nDim+nRow] = pMatrix[nRow*nDim+nCol];
 
		for(int i = 0; i < nDim; i ++) 
		{ 
			if((i!=nCol) && (i!=nRow)) 
			{ 
				int u = i*nDim + nRow;	//p  
				int w = i*nDim + nCol;	//q
				dbMax = pMatrix[u]; 
				pMatrix[u]= pMatrix[w]*dbSinTheta + dbMax*dbCosTheta; 
				pMatrix[w]= pMatrix[w]*dbCosTheta - dbMax*dbSinTheta; 
			} 
		} 
 
		for (int j = 0; j < nDim; j ++)
		{
			if((j!=nCol) && (j!=nRow)) 
			{ 
				int u = nRow*nDim + j;	//p
				int w = nCol*nDim + j;	//q
				dbMax = pMatrix[u]; 
				pMatrix[u]= pMatrix[w]*dbSinTheta + dbMax*dbCosTheta; 
				pMatrix[w]= pMatrix[w]*dbCosTheta - dbMax*dbSinTheta; 
			} 
		}
 
		//计算特征向量
		for(int i = 0; i < nDim; i ++) 
		{ 
			int u = i*nDim + nRow;		//p   
			int w = i*nDim + nCol;		//q
			dbMax = pdblVects[u]; 
			pdblVects[u] = pdblVects[w]*dbSinTheta + dbMax*dbCosTheta; 
			pdblVects[w] = pdblVects[w]*dbCosTheta - dbMax*dbSinTheta; 
		} 
 
	}
    
    int min_id = 0;
	float minEigenvalue;

	for(int i = 0; i < nDim; i ++) 
	{   
		pdbEigenValues[i] = pMatrix[i*nDim+i];
        if(i == 0)
            minEigenvalue = pdbEigenValues[i];
        else
        {
            if(minEigenvalue > pdbEigenValues[i])
            {
                minEigenvalue = pdbEigenValues[i];
                min_id = i;	
            }
        }
    } 

    for(int i = 0; i < nDim; i ++) 
    {  
        maxvector[i] = pdblVects[min_id + nDim * i];
    }
}

//geographic location to memory location index
__device__ int dev_IndexToRange(int *cell_p){
    int d_index[2];
    d_index[0] = (cell_p[0] - start_indice[0] + Length)%Length;
    d_index[1] = (cell_p[1] - start_indice[1] + Length)%Length;
    int index = d_index[0] * Length + d_index[1];
    return index;
}

__global__ void G_Init_map()
{    
    int i = blockDim.x * blockIdx.x + threadIdx.x; 
    
	if (i < Length * Length ) {
        map_intensity[i] = 0;
        map_elevation[i] = -10;
        map_variance[i] = -10;
        map_lowest[i] = 100;
        map_traver[i] = -10;
        map_colorR[i] = 0;
        map_colorG[i] = 0;
        map_colorB[i] = 0;

    }

}

__global__ void G_Clear_allmap()
{    
    int i = blockDim.x * blockIdx.x + threadIdx.x; 
	if (i < Length * Length) {
        map_intensity[i] = 0;
        map_elevation[i] = -10;
        map_variance[i] = -10;
        map_traver[i] = -10;
        map_colorR[i] = 0;
        map_colorG[i] = 0;
        map_colorB[i] = 0;
        //printf("%f ", map_elevation[i]);
    }
  
}

__global__ void G_Clear_maplowest()
{    
    int i = blockDim.x * blockIdx.x + threadIdx.x; 
	if (i < Length * Length) {
        map_lowest[i] = 10; 
        //printf("%f ", map_elevation[i]);
    }
}

__global__ void G_Printf_map()
{    
    
    for(int i = 0; i < Length; i++)
        for(int j = 0; j < Length; j++)
        {
            printf("%f ", map_elevation[i * Length + j]);
            
        }
    printf("P:start_indice:%d,%d\n",start_indice[0], start_indice[1]);
    printf("P:central_coordinate:%f,%f\n",central_coordinate[0], central_coordinate[1]);

}

__global__ void G_Clear_map(int start, int shift, bool row_flag)
{    
    int i = blockDim.x * blockIdx.x + threadIdx.x; 
	if (i < Length * shift) {
        if(row_flag){
            map_intensity[start * Length + i] = 0;
            map_elevation[start * Length + i] = -10;
            map_variance[start * Length + i] = -10;
            map_colorR[start * Length + i] = 0;
            map_colorG[start * Length + i] = 0;
            map_colorB[start * Length + i] = 0;
        }
        else{
            map_intensity[i / shift * Length + i % shift + start] = 0;
            map_elevation[i / shift * Length + i % shift + start] = -10;
            map_variance[i / shift * Length + i % shift + start] = -10;
            map_colorR[i / shift * Length + i % shift + start] = 0;
            map_colorG[i / shift * Length + i % shift + start] = 0;
            map_colorB[i / shift * Length + i % shift + start] = 0;
        }
    }
}

__device__ Eigen::Vector3f cuda_Transpose(Eigen::RowVector3f A)
{
	Eigen::Vector3f A_T;
	A_T(0,0) = A(0,0);
	A_T(1,0) = A(0,1);
	A_T(2,0) = A(0,2);
	return A_T;
}

template<typename PrimType_>
//inline static 
__device__ Eigen::Matrix<PrimType_, 3, 3> GetSkewMatrixFromVector(const Eigen::Matrix<PrimType_, 3, 1>& vec) {
  Eigen::Matrix<PrimType_, 3, 3> mat;
  mat << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
  return mat;
}

__device__ float cuda_computer(Eigen::RowVector3f A, Eigen::Matrix3f B, Eigen::Vector3f C)
{
	Eigen::RowVector3f A1 = A * B;
	float result = A1(0,0) * C(0,0) + A1(0,1) * C(1,0) + A1(0,2) * C(2,0);
	return result;
}

__device__ Eigen::Matrix3f SkewMatrixFromVector(Eigen::Vector3f vec)
{
	Eigen::Matrix<float, 3, 3> mat;
  	mat << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
	return mat;
}

__device__ int PointsToIndex(float p_x, float p_y)
{
    float shift_x = p_x - central_coordinate[0];
    float shift_y = p_y - central_coordinate[1];
    int index_x, index_y, index;
    if(Length % 2 == 0)
    {
        index_x = (int)((float)(Length / 2) - shift_x / Resolution) ;
        index_y = (int)((float)(Length / 2) - shift_y / Resolution);
    }
    else
    {
        index_x = Length / 2 - static_cast<int>(shift_x / Resolution +0.5 * (shift_x > 0 ? 1 : -1));
        index_y = Length / 2 - static_cast<int>(shift_y / Resolution +0.5 * (shift_y > 0 ? 1 : -1));
    }

    if(index_x >= 0 && index_x < Length && index_y >= 0 && index_y < Length)
        index = index_x * Length + index_y;
    else
        index = -1;
    return index;
}

__device__ int PointsToMapIndex(float p_x, float p_y)
{
    float shift_x = p_x - central_coordinate[0];
    float shift_y = p_y - central_coordinate[1];
    int index_x, index_y, index;
    if(Length % 2 == 0)
    {
        index_x = (int)((float)(Length / 2) - shift_x / Resolution) ;
        index_y = (int)((float)(Length / 2) - shift_y / Resolution);
    }
    else
    {
        index_x = Length / 2 - static_cast<int>(shift_x / Resolution +0.5 * (shift_x > 0 ? 1 : -1));
        index_y = Length / 2 - static_cast<int>(shift_y / Resolution +0.5 * (shift_y > 0 ? 1 : -1));
    }
    
    if(index_x >= 0 && index_x < Length && index_y >= 0 && index_y < Length)
    {
        int storage_x = (index_x + start_indice[0]) % Length;
        int storage_y = (index_y + start_indice[1]) % Length;
    
        index = storage_x * Length + storage_y;
    }
    else
        index = -1;
    return index;
}

__device__ static float atomicMax(float* address, float val) 
{ 
    int* address_as_i = (int*) address; 
    int old = *address_as_i, assumed; 
    do { 
     assumed = old; 
     old = ::atomicCAS(address_as_i, assumed, 
      __float_as_int(::fmaxf(val, __int_as_float(assumed)))); 
    } while (assumed != old); 
    return __int_as_float(old); 
} 

__device__ static float atomicMin(float* address, float val) 
{ 
    int* address_as_i = (int*) address; 
    int old = *address_as_i, assumed; 
    do { 
     assumed = old; 
     old = ::atomicCAS(address_as_i, assumed, 
      __float_as_int(::fminf(val, __int_as_float(assumed)))); 
    } while (assumed != old); 
    return __int_as_float(old); 
} 

__global__ void G_pointsprocess(int* map_index, float *point_x, float *point_y, float *point_z, float *result_var, float *point_x_ts, float *point_y_ts, float *point_z_ts, Eigen::Matrix4f transform, int point_num, Eigen::RowVector3f C_sensorJacobian, Eigen::Matrix3f C_rotationVariance, Eigen::Matrix3f C_C_SB_transpose, Eigen::RowVector3f C_P_mul_C_BM_transpose, Eigen::Matrix3f C_B_r_BS_skew)
{
    int i = blockDim.x * blockIdx.x + threadIdx.x; 
	if(i < point_num)
	{
		float point_height =  transform(2, 0) * point_x[i] + transform(2, 1) * point_y[i] + transform(2, 2) * point_z[i] + transform(2, 3);
		
		int flag = 0;
        // !!! IMPORTANT FILTER PARAMETERS
		if((point_x[i] > -1.5 && point_x[i] < 1.5 && point_y[i] > -1.5 && point_y[i] < 1.5) || (point_y[i] > -1 && point_y[i] < 1) || point_y[i] > 0)
		{
			flag = 1;
		}
		if((point_height > C_relativeLowerThreshold && point_height < C_relativeUpperThreshold) && flag == 0)
		{
			point_x_ts[i] = transform(0, 0) * point_x[i] + transform(0, 1) * point_y[i] + transform(0, 2) * point_z[i] + transform(0, 3) ;
			point_y_ts[i] = transform(1, 0) * point_x[i] + transform(1, 1) * point_y[i] + transform(1, 2) * point_z[i] + transform(1, 3) ;
			point_z_ts[i] = point_height;

			Eigen::Vector3f pointVector(point_x[i], point_y[i], point_z[i]); // S_r_SP
			float heightVariance = 0.0; // sigma_p

			// Measurement distance.
			float measurementDistance = pointVector.norm();

			// Compute sensor covariance matrix (Sigma_S) with sensor model.
			float varianceNormal = pow(C_min_r, 2);
			float varianceLateral = pow(C_beam_c + C_beam_a * measurementDistance, 2);

			Eigen::Matrix3f sensorVariance = Eigen::Matrix3f::Zero();
			sensorVariance.diagonal() << varianceLateral, varianceLateral, varianceNormal;

			// Robot rotation Jacobian (J_q).
			const Eigen::Matrix3f C_SB_transpose_times_S_r_SP_skew = SkewMatrixFromVector(Eigen::Vector3f(C_C_SB_transpose * pointVector));
			Eigen::RowVector3f rotationJacobian = C_P_mul_C_BM_transpose * (C_SB_transpose_times_S_r_SP_skew + C_B_r_BS_skew);
		
			// Measurement variance for map (error propagation law).
			Eigen::Vector3f rotationJacobian_T = cuda_Transpose(rotationJacobian);
			heightVariance = cuda_computer(rotationJacobian, C_rotationVariance, rotationJacobian_T);
		
			Eigen::Vector3f C_sensorJacobian_T = cuda_Transpose(C_sensorJacobian);
			heightVariance += cuda_computer(C_sensorJacobian, sensorVariance, C_sensorJacobian_T);
	
			// Copy to list.
            result_var[i] = heightVariance;
            
            int grid_index = PointsToIndex(point_x_ts[i], point_y_ts[i]);
            map_index[i] = PointsToMapIndex(point_x_ts[i], point_y_ts[i]);
            if(grid_index != -1)
            {
                atomicMin(&map_lowest[grid_index], point_height);
                if(point_height == map_lowest[grid_index])
                {
                    map_lowest[grid_index] = map_lowest[grid_index] + 3 * heightVariance; 
                }
            }
		}
		else
		{
            map_index[i] = -1;
			point_x[i] = -1;
			point_y[i] = -1;
			point_z[i] = -1;
			point_x_ts[i] = -1;
			point_y_ts[i] = -1;
			point_z_ts[i] = -1;
			result_var[i] = -1;
		}
    }
    // Debug
    // printf("GPU points:i:%d;x:%f;y:%f;z:%f", i, point_x_ts[i], point_y_ts[i], point_z_ts[i]);
}

__global__ void G_get_mapinfo(int cell_num, float *dev_map_ele, float *dev_map_var)
{
    int i = blockDim.x * blockIdx.x + threadIdx.x; 
	if(i < cell_num)
	{
        dev_map_ele[i]  = map_elevation[i];
        dev_map_var[i]  = map_variance[i];
    }
}

__global__ void G_set_mapinfo(int cell_num, float *dev_map_ele, float *dev_map_var)
{
    int i = blockDim.x * blockIdx.x + threadIdx.x; 
	if(i < cell_num)
	{
        map_elevation[i] = dev_map_ele[i];
        map_variance[i]  = dev_map_var[i];
    }
}

__global__ void G_fuse(int *point_index, int *points_colorR, int *points_colorG, int *points_colorB, float* points_intensity, float *points_h, float *points_v, int point_num){
    int map_index = blockDim.x * blockIdx.x + threadIdx.x; 
    if(map_index < Length * Length){
       for(int i = 0; i < point_num; i++)
       {
            if(point_index[i] != map_index || points_h[i] == -1)
                continue;
            if (map_elevation[map_index]== -10){
                // No prior information in elevation map, use measurement.
                map_elevation[map_index] = points_h[i];
                map_variance[map_index] = points_v[i];
                if(points_colorR[i] != 0 && points_colorG[i] != 0 && points_colorB[i] != 0 && points_intensity[i] != 0)
                {
                    map_intensity[map_index] = points_intensity[i];
                    map_colorR[map_index] = points_colorR[i];
                    map_colorG[map_index] = points_colorG[i];
                    map_colorB[map_index] = points_colorB[i];
                } 
            }
            else{
            // Deal with multiple heights in one cell.
            // Debug fabs,sqrt！！！！！！！！！！！！！！！！！！
            // printf("points height:%f;points var:%f;point_num:%d;cells height:%f, map_variance:%f\n", points_max[i], points_var[i], points_num[i], map_elevation[map_index], map_variance[map_index]);
                const float mahalanobisDistance = fabs(points_h[i] - map_elevation[map_index]) / sqrt(map_variance[map_index]);
                //printf("mahalanobisDistance:%f\n", mahalanobisDistance);
                if (mahalanobisDistance > 5) {
                    if (map_elevation[map_index] < points_h[i]) {
                        map_elevation[map_index] = points_h[i];
                        map_variance[map_index] = points_v[i];
                        if(points_colorR[i] != 0 && points_colorG[i] != 0 && points_colorB[i] != 0 && points_intensity[i] != 0)
                        {
                            map_intensity[map_index] = points_intensity[i];
                            map_colorR[map_index] = points_colorR[i];
                            map_colorG[map_index] = points_colorG[i];
                            map_colorB[map_index] = points_colorB[i];
                        }
                    }
                }
                else{
                    map_elevation[map_index] = (map_variance[map_index] * points_h[i] + points_v[i] * map_elevation[map_index]) / (map_variance[map_index] + points_v[i]);
                    map_variance[map_index] = (points_v[i] * map_variance[map_index]) / (points_v[i] + map_variance[map_index]);
                    if(points_colorR[i] != 0 && points_colorG[i] != 0 && points_colorB[i] != 0 && points_intensity[i] != 0)
                    {
                        map_intensity[map_index] = points_intensity[i];
                        map_colorR[map_index] = points_colorR[i];
                        map_colorG[map_index] = points_colorG[i];
                        map_colorB[map_index] = points_colorB[i];
                    }
                    // Debug
                    //printf("point_var:%f, map_variance:%f\n", points_v[i], map_variance[map_index]);
                }  
            }
       }

       if(map_variance[map_index] < 0.0001)
            map_variance[map_index] = 0.0001;

    }
}

//计算每块地图的X，Y,XY方差
__global__ void G_Mapvar_update(float var_update)
{
	int i = blockDim.x * blockIdx.x + threadIdx.x; 
	if (i < Length * Length) {
        if(map_variance[i] != -10)
            map_variance[i] += var_update;
    } 
}

__global__ void G_Mapfeature(int *d_colorR, int *d_colorG, int *d_colorB, float *d_elevation, float *d_var, float *d_rough, float *d_slope, float *d_traver, float *d_intensity)
{
    
    int idx = threadIdx.x + blockDim.x * blockIdx.x;
    if(idx >= Length * Length) return;

    Pos3 point[25];

    float s_z;
    float px_mean = 0;
    float py_mean = 0;
    float pz_mean = 0;
    
    int cell_x = idx / Length;
    int cell_y = idx % Length;
    int point_x;
    int point_y;

    int Ele_x;
    int Ele_y;

    // Debug
    //slope[idx] = map_height[idx];
    //printf("X:%d,Y:%d,height:%f", cell_x, cell_y, map_height[idx]);

    d_elevation[idx] = map_elevation[idx];
    d_colorR[idx] = map_colorR[idx];
    d_colorG[idx] = map_colorG[idx];
    d_colorB[idx] = map_colorB[idx];
    d_intensity[idx] = map_intensity[idx];

    d_var[idx] = map_variance[idx];
    if(map_elevation[idx] == -10)
        return ;
    
    int p_n = 0;    
    for (int i = -2; i < 3 ;i ++)
    {
      for(int j = -2; j < 3; j++)
      {
            Ele_x = (cell_x + Length - start_indice[0]) % Length;
            Ele_y = (cell_y + Length - start_indice[1]) % Length;

            Ele_x = Ele_x + i;
            Ele_y = Ele_y + j;

            if( Ele_x >= 0 && Ele_x < Length && Ele_y >= 0 && Ele_y < Length)
            {

                point_x = cell_x + i;
                point_y = cell_y + j;

                point_x = (point_x + Length) % Length;
                point_y = (point_y + Length) % Length;
                s_z = map_elevation[point_x * Length + point_y];
                if(s_z != -10)
                {
                    point[p_n].x = point_x * Resolution;
                    point[p_n].y = point_y * Resolution;
                    point[p_n].z = s_z;

                    px_mean = px_mean + point[p_n].x;
                    py_mean = py_mean + point[p_n].y;
                    pz_mean = pz_mean + point[p_n].z;
                    p_n++;
                }
            }
      }
    }
    
    if(p_n > 7)
    {
        px_mean = px_mean / p_n;
        py_mean = py_mean / p_n;
        pz_mean = pz_mean / p_n;
    
        float pMatrix[9] = {0};
        for(int i = 0; i < p_n; i ++)
        {
            pMatrix[0] = pMatrix[0] + (point[i].x - px_mean) * (point[i].x - px_mean);
            pMatrix[4] = pMatrix[4] + (point[i].y - py_mean) * (point[i].y - py_mean);
            pMatrix[8] = pMatrix[8] + (point[i].z - pz_mean) * (point[i].z - pz_mean);
            pMatrix[1] = pMatrix[1] + (point[i].x - px_mean) * (point[i].y - py_mean);
            pMatrix[2] = pMatrix[2] + (point[i].x - px_mean) * (point[i].z - pz_mean);
            pMatrix[5] = pMatrix[5] + (point[i].y - py_mean) * (point[i].z - pz_mean);
            pMatrix[3] = pMatrix[1];
            pMatrix[6] = pMatrix[2];
            pMatrix[7] = pMatrix[5];
        }
        
        float dbEps = 0.01;
        int nJt = 30;
        int nDim = 3;
        float normal_vec[3];
        float Slope;
        computerEigenvalue(pMatrix, nDim, normal_vec, dbEps, nJt);
        
        float height = map_elevation[idx];
        float smooth_height = pz_mean;
        
        if(normal_vec[2] > 0)
            Slope = acos(normal_vec[2]);
        else
            Slope = acos(-normal_vec[2]); 
      
        float Rough = fabs(height - smooth_height);
        float Traver = 0.5 * (1.0 - Slope / 0.6)+ 0.5 * (1.0 - (Rough / 0.2));
        d_slope[idx] = Slope; 
        d_rough[idx] = Rough;

        d_traver[idx] = Traver;
        map_traver[idx] = Traver;
    }
    else
    {
        d_slope[idx] = 0; 
        d_rough[idx] = 0;
        d_traver[idx] = -10;
        map_traver[idx] = -10;
    }
    
}

__device__ void StorageP2geoP(int index_s_x, int index_s_y, int *index_g){
    index_g[0] = (index_s_x + Length - start_indice[0]) % Length;
    index_g[1] = (index_s_y + Length - start_indice[1]) % Length;
}

__device__ int Storageindex(int index_g_x, int index_g_y){
    int index_s = index_g_x * Length + index_g_y;
    return index_s;
}

__device__ bool P_isVaild(int cell_index_x, int cell_index_y)
{
    int storage_index = Storageindex(cell_index_x, cell_index_y); 
    //d_map_clean[storage_index] = 0;
    if(map_lowest[storage_index] == 10)
        return false;
    else
        return true;
}

__device__ float d_min_elevation(int cell_index_x, int cell_index_y, int obstacle_index_x, float robot_index_x)
{
    float x1 = (float)(cell_index_x - obstacle_index_x);
    float x2 = (float)cell_index_x - robot_index_x;
    int storage_index = Storageindex(cell_index_x, cell_index_y); 
    
    // Debug
    // map_lowest[storage_index] = 1;
    // d_map_clean[storage_index] = 0;
    
    float h2 = sensorZatLowestScan - map_lowest[storage_index]; 
    float obstacle_max_ele = map_lowest[storage_index] + h2 / x2 * x1;
    
    return obstacle_max_ele;
}

__global__ void G_Raytracing()
{
    int i = blockDim.x * blockIdx.x + threadIdx.x; 
	if (i < Length * Length){
        if(map_traver[i] < obstacle_threshold && map_elevation[i] != -10)
        {
            // Debug
            // printf("i:%d,map_traver:%f\n", i, map_traver[i]);
            // d_map_clean[i] = -1;
            int cell_x = i / Length;
            int cell_y = i % Length;
            
            int robot_index;
            int obstacle_indice[2];
            StorageP2geoP(cell_x, cell_y, obstacle_indice);

            float obstacle_ele = map_elevation[i];
            int current_indice[2];
            float increment[2];
            int increment_x, increment_y;
            current_indice[0] = obstacle_indice[0]; 
            current_indice[1] = obstacle_indice[1]; 
           
            if(Length % 2 == 0)
            {
                robot_index = (float)(Length / 2 - 0.5);
                increment[0] = obstacle_indice[0] - robot_index;
                increment[1] = obstacle_indice[1] - robot_index;
            }
            else
            {
                robot_index = (float)(Length / 2);
                increment[0] = obstacle_indice[0] - robot_index;
                increment[1] = obstacle_indice[1] - robot_index;
            }

            if(increment[0] > 0)
                increment_x = 1;
            else if(increment[0] == 0)
                increment_x = 0;
            else
                increment_x = -1;    
            
            if(increment[1] > 0)
                increment_y = 1;
            else if(increment[1] == 0)
                increment_y = 0;
            else
                increment_y = -1;   
            
            float obstacle_max_ele;
            float obstacle_restrict_ele = obstacle_ele;
            if(increment_x == 0 && increment_y == 0)
                return ;
            else if(increment_x == 0)
            {
                current_indice[1] += increment_y;
                while(current_indice[1] >= 0 && current_indice[1] < Length)
                {
                    if(P_isVaild(current_indice[0], current_indice[1]))
                    {
                        obstacle_max_ele = d_min_elevation(current_indice[0], current_indice[1], obstacle_indice[0], robot_index); 
                         //std::cout << "x:"<<current_indice[0] << " y:" << current_indice[1]<< std::endl;   
                         if(obstacle_max_ele < obstacle_restrict_ele)
                            obstacle_restrict_ele = obstacle_max_ele;
                    }
                    current_indice[1] += increment_y;
                }
                return ;
            }
            else if(increment_y == 0)
            {
                current_indice[0] += increment_x;
                while(current_indice[0] >= 0 && current_indice[0] < Length)
                {
                    if(P_isVaild(current_indice[0], current_indice[1]))
                    {
                        obstacle_max_ele = d_min_elevation(current_indice[0], current_indice[1], obstacle_indice[0], robot_index);    
                    //std::cout << "x:"<<current_indice[0] << " y:" << current_indice[1]<< std::endl;  
                        if(obstacle_max_ele < obstacle_restrict_ele)
                            obstacle_restrict_ele = obstacle_max_ele; 
                    }    
                current_indice[0] += increment_x;
                }
                return ;
            }

            float dis = sqrt(increment[0] * increment[0] + increment[1] * increment[1]);
            float dir[2];
            dir[0] = increment[0] /dis;
            dir[1] = increment[1] /dis;
            
            float threshold;
            if(fabs(increment[0]) > fabs(increment[1]))
                threshold = sqrt(0.5 * 0.5 + pow(0.5/increment[0]*increment[1], 2));
            else
                threshold = sqrt(0.5 * 0.5 + pow(0.5/increment[1]*increment[0], 2));
            

            float dir_num_x;
            float dir_num_y;

            float bound_increment_x = (float)increment_x/2;
            float bound_increment_y = (float)increment_y/2;

            dir_num_x = bound_increment_x / dir[0];
            dir_num_y = bound_increment_y / dir[1];

            float dir_num_later = 0;
          
            // Debug
            // std::cout << "threshold" << threshold << std::endl;
            // std::cout << "x:"<<current_indice[0] << " y:" << current_indice[1]<< std::endl; 
            while(current_indice[0] >= 0 && current_indice[0] < Length && current_indice[1] >= 0 && current_indice[1] < Length)
            {
                if(dir_num_x > dir_num_y)
                {
                    if(dir_num_y - dir_num_later > threshold && current_indice[0] != obstacle_indice[0] && current_indice[1] != obstacle_indice[1])
                    {
                        if(P_isVaild(current_indice[0], current_indice[1]))
                        {
                            obstacle_max_ele = d_min_elevation(current_indice[0], current_indice[1], obstacle_indice[0], robot_index);    
                            if(obstacle_max_ele < obstacle_restrict_ele)
                                obstacle_restrict_ele = obstacle_max_ele;
                            //std::cout << "x:"<<current_indice[0] << " y:" << current_indice[1]<< std::endl;
                        }  
                    }
                
                    current_indice[1] += increment_y;
                    bound_increment_y += (float)increment_y;
                    dir_num_later = dir_num_y;
                    dir_num_y = bound_increment_y / dir[1];         
                }
                else if(dir_num_x < dir_num_y)
                {
                    if(dir_num_x - dir_num_later > threshold && current_indice[0] != obstacle_indice[0] && current_indice[1] != obstacle_indice[1])
                    {
                        if(P_isVaild(current_indice[0], current_indice[1]))
                        {
                            obstacle_max_ele = d_min_elevation(current_indice[0], current_indice[1], obstacle_indice[0], robot_index);    
                            if(obstacle_max_ele < obstacle_restrict_ele)
                                obstacle_restrict_ele = obstacle_max_ele; 
                            // Debug 
                            // std::cout << "x:"<<current_indice[0] << " y:" << current_indice[1]<< std::endl;
                        } 
                    }
                    current_indice[0] += increment_x;  
                    bound_increment_x += (float)increment_x;
                    dir_num_later = dir_num_x;
                    dir_num_x = bound_increment_x / dir[0];
                }
                else
                {
                    if(dir_num_x - dir_num_later > threshold && current_indice[0] != obstacle_indice[0] && current_indice[1] != obstacle_indice[1])
                    {
                        if(P_isVaild(current_indice[0], current_indice[1]))
                        {
                            obstacle_max_ele = d_min_elevation(current_indice[0], current_indice[1], obstacle_indice[0], robot_index);    
                            if(obstacle_max_ele < obstacle_restrict_ele)
                                obstacle_restrict_ele = obstacle_max_ele;  
                        // Debug
                        // std::cout << "x:"<<current_indice[0] << " y:" << current_indice[1]<< std::endl;
                        } 
                    }
                    current_indice[0] += increment_x;  
                    current_indice[1] += increment_y;  
                    bound_increment_x += (float)increment_x;
                    bound_increment_y += (float)increment_y;
                    dir_num_later = dir_num_x;
                    dir_num_x = bound_increment_x / dir[0];
                    dir_num_y = bound_increment_y / dir[1];
                    
                }
            }
            // Debug
            // printf("height:%f,restrict_height:%f\n", obstacle_ele, obstacle_restrict_ele);
            // d_map_clean[i] = obstacle_restrict_ele;
            if(obstacle_ele - 3 * sqrt(map_variance[i])> obstacle_restrict_ele)
                map_elevation[i] = -10;
        }
        //else
        //    d_map_clean[i] = 1;
    }
}

bool getIndexShiftFromPositionShift(int *indexShift,
    float *positionShift, float resolution)
{
    for (int i = 0; i < 2; i++) {
        indexShift[i] = static_cast<int>(positionShift[i] / resolution + 0.5 * (positionShift[i] > 0 ? 1 : -1));
    }
    // Debug
    // std::cout << "indexShift1:" << indexShift[0]<< "   indexShift2:" << indexShift[1]<< std::endl;
    return true;
}

bool getPositionShiftFromIndexShift(float *positionShift,
    int *indexShift, float resolution)
{
    for(int i = 0; i < 2; i++)
    { 
        positionShift[i] = (float)indexShift[i] * resolution;
    }
    // Debug
    // std::cout << "positionShift1:" << positionShift[0]<<" positionShift2：" << positionShift[1]<< std::endl;
    return true;
}

int IndexToRange(int index,int Length)
{
    if (index < 0) index += ((-index / Length) + 1) * Length;
    index = index % Length;
    return index;
}

void Clear_regionrow(int Start_x, int Shift_x, int length)
{
    int cell_num = Shift_x * length;
    int threadsPerBlock = 256; 
    int blocksPerGrid =(cell_num + threadsPerBlock - 1) / threadsPerBlock; 
    G_Clear_map<<<blocksPerGrid, threadsPerBlock>>>(Start_x, Shift_x, true);
}

void Clear_regioncol(int Start_y, int Shift_y, int length)
{
    int cell_num = Shift_y * length;
    int threadsPerBlock = 256; 
    int blocksPerGrid =(cell_num + threadsPerBlock - 1) / threadsPerBlock; 
    G_Clear_map<<<blocksPerGrid, threadsPerBlock>>>(Start_y, Shift_y, false);
    
}

void Init_GPU_elevationmap(int length, float resolution, float h_mahalanobisDistanceThreshold_, float h_obstacle_threshold)
{
    float init_centralcoordinate[2] = {0, 0};
    int init_startindice[2] = {0, 0};
    
    float *h_map_intensity;
    float *h_map_elevation;
    float *h_map_variance;
    float *h_map_lowest;
    float *h_map_traver;
    int *h_map_colorR;
    int *h_map_colorG;
    int *h_map_colorB;

    cudaMalloc((void**)&h_map_intensity, sizeof(float) * length * length);
    cudaMalloc((void**)&h_map_elevation, sizeof(float) * length * length);
    cudaMalloc((void**)&h_map_variance, sizeof(float) * length * length);
    cudaMalloc((void**)&h_map_lowest, sizeof(float) * length * length);
    cudaMalloc((void**)&h_map_traver, sizeof(float) * length * length);
    cudaMalloc((void**)&h_map_colorR, sizeof(int) * length * length);
    cudaMalloc((void**)&h_map_colorG, sizeof(int) * length * length);
    cudaMalloc((void**)&h_map_colorB, sizeof(int) * length * length);

    cudaMemcpyToSymbol(map_intensity, &h_map_intensity, sizeof(float *),size_t(0), cudaMemcpyHostToDevice);  
    cudaMemcpyToSymbol(map_elevation, &h_map_elevation, sizeof(float *),size_t(0), cudaMemcpyHostToDevice);  
    cudaMemcpyToSymbol(map_variance, &h_map_variance, sizeof(float *),size_t(0), cudaMemcpyHostToDevice);  
    cudaMemcpyToSymbol(map_lowest, &h_map_lowest, sizeof(float *),size_t(0), cudaMemcpyHostToDevice);  
    cudaMemcpyToSymbol(map_traver, &h_map_traver, sizeof(float *),size_t(0), cudaMemcpyHostToDevice);  
    cudaMemcpyToSymbol(map_colorR, &h_map_colorR, sizeof(int *),size_t(0), cudaMemcpyHostToDevice); 
    cudaMemcpyToSymbol(map_colorG, &h_map_colorG, sizeof(int *),size_t(0), cudaMemcpyHostToDevice); 
    cudaMemcpyToSymbol(map_colorB, &h_map_colorB, sizeof(int *),size_t(0), cudaMemcpyHostToDevice); 

    cudaMemcpyToSymbol(central_coordinate, &init_centralcoordinate, 2*sizeof(float));
    cudaMemcpyToSymbol(start_indice, &init_startindice, 2*sizeof(int));

    cudaMemcpyToSymbol(Length, &length, sizeof(int));
    cudaMemcpyToSymbol(Resolution, &resolution, sizeof(float));
    cudaMemcpyToSymbol(mahalanobisDistanceThreshold_, &h_mahalanobisDistanceThreshold_, sizeof(float));
    cudaMemcpyToSymbol(obstacle_threshold, &h_obstacle_threshold, sizeof(float));
    
    int threadsPerBlock = 256; 
    int blocksPerGrid =(length * length + threadsPerBlock - 1) / threadsPerBlock; 
    std::cout << "GPU Init mapping:"<< length * length <<std::endl;
   
    G_Init_map<<<blocksPerGrid, threadsPerBlock>>>();
    //G_Printf_map<<<1, 1>>>();

    cudaError_t cudaStatus = cudaGetLastError();
    if (cudaStatus != cudaSuccess) 
    {
        fprintf(stderr, "addKernel launch failed: %s\n", cudaGetErrorString(cudaStatus));
        //goto Error;
    }
    //cudaDeviceSynchronize();
} 

float PositionToRange(float p, float shift, float resolution)
{
    int p_index = round(p /resolution);
    int shift_index = round(shift /resolution);
    int current_index = p_index + shift_index;
    return (current_index * resolution);
}

void Move(float *current_Position, float resolution, int length, float *Central_coordinate, int *Start_indice, float *alignedPositionShift)
{
    int indexShift[2];
    float positionShift[2];
    
    float h_central_coordinate[2];
    int h_start_indice[2];
    float robot_height = current_Position[2];
    cudaMemcpyToSymbol(sensorZatLowestScan, &robot_height, sizeof(float));

    cudaMemcpyFromSymbol(&h_central_coordinate, central_coordinate, sizeof(float) * 2);
    cudaMemcpyFromSymbol(&h_start_indice, start_indice, sizeof(int) * 2);
   
    positionShift[0] = current_Position[0] - h_central_coordinate[0];
    positionShift[1] = current_Position[1] - h_central_coordinate[1];

    // Debug
    // std::cout << "start_indicex:" <<h_start_indice[0] <<"   2:" <<h_start_indice[1]<<std::endl;
    // std::cout << "current_x:" <<current_Position[0] <<"   2:" <<current_Position[1]<<std::endl;
    // std::cout << "later_x:" <<h_central_coordinate[0] <<"   2:" <<h_central_coordinate[1]<<std::endl;
    // std::cout << "position_shift 1:" <<positionShift[0] <<"   2:" <<positionShift[1]<<std::endl;

    getIndexShiftFromPositionShift(indexShift, positionShift, resolution);
    // float alignedPositionShift[2];
    getPositionShiftFromIndexShift(alignedPositionShift, indexShift, resolution);
    
    for(int i = 0; i < 2; i++)
    {
        if(indexShift[i] != 0){
            if(indexShift[i] >= length)
            {
                int threadsPerBlock = 128; 
                int blocksPerGrid =(length * length + threadsPerBlock - 1) / threadsPerBlock; 
                G_Clear_allmap<<<blocksPerGrid, threadsPerBlock>>>();
            }
            else
            {
                int sign = (indexShift[i] > 0 ? 1 : -1);
                int startIndex = h_start_indice[i] - ( sign > 0 ? 1:0 );
                int endIndex = startIndex + sign - indexShift[i];
                int nCells = abs(indexShift[i]);
                int index = (sign < 0 ? startIndex : endIndex);
                index = IndexToRange(index, length);
                if(index + nCells <= length){
                    if(i == 0)
                        Clear_regionrow(index, nCells, length);
                    else
                        Clear_regioncol(index, nCells, length); 
                }
                else{
                    int firstIndex = index;
                    int firstnCells = length - firstIndex;
                    if(i == 0)
                        Clear_regionrow(firstIndex, firstnCells, length);
                    else
                        Clear_regioncol(firstIndex, firstnCells, length);
                    
                    int secondIndex = 0;
                    int secondnCells = nCells - firstnCells;
                    if(i == 0)
                        Clear_regionrow(secondIndex, secondnCells, length);
                    else
                        Clear_regioncol(secondIndex, secondnCells, length);
                }
            }
        }
        h_start_indice[i] -= indexShift[i];
        h_start_indice[i] = IndexToRange(h_start_indice[i], length);
        h_central_coordinate[i] = PositionToRange(h_central_coordinate[i], alignedPositionShift[i], resolution);
    }   
   
    cudaMemcpyToSymbol(start_indice, &h_start_indice, sizeof(int) * 2);
    cudaMemcpyToSymbol(central_coordinate, &h_central_coordinate, sizeof(float) * 2);
    
    Central_coordinate[0] = h_central_coordinate[0];
    Central_coordinate[1] = h_central_coordinate[1];
    Start_indice[0] = h_start_indice[0];
    Start_indice[1] = h_start_indice[1];
    //cudaDeviceSynchronize();
}

int Process_points(int *map_index, float *point_x, float *point_y, float *point_z, float *point_var, float *point_x_ts, float *point_y_ts, float *point_z_ts, Eigen::Matrix4f transform, int point_num, double relativeLowerThreshold, double relativeUpperThreshold, float min_r, float beam_a, float beam_c, Eigen::RowVector3f sensorJacobian, Eigen::Matrix3f rotationVariance, Eigen::Matrix3f C_SB_transpose, Eigen::RowVector3f P_mul_C_BM_transpose, Eigen::Matrix3f B_r_BS_skew)
{
	float* dev_x; 
	float* dev_y; 
	float* dev_z; 
	float* dev_result;
	float* dev_x_ts; 
	float* dev_y_ts; 
    float* dev_z_ts; 
    int* dev_mapindex;

	cudaMalloc((void**)&dev_x, point_num * sizeof(float)); 
	cudaMalloc((void**)&dev_y, point_num * sizeof(float)); 
    cudaMalloc((void**)&dev_z, point_num * sizeof(float));
    
	cudaMalloc((void**)&dev_result, point_num * sizeof(float));
	cudaMalloc((void**)&dev_x_ts, point_num * sizeof(float)); 
	cudaMalloc((void**)&dev_y_ts, point_num * sizeof(float)); 
	cudaMalloc((void**)&dev_z_ts, point_num * sizeof(float));
    cudaMalloc((void**)&dev_mapindex, point_num * sizeof(int));

	cudaMemcpy(dev_x, point_x, point_num * sizeof(float), cudaMemcpyHostToDevice); 
	cudaMemcpy(dev_y, point_y, point_num * sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(dev_z, point_z, point_num * sizeof(float), cudaMemcpyHostToDevice);

	cudaMemcpyToSymbol(C_relativeLowerThreshold, &relativeLowerThreshold, sizeof(double));
	cudaMemcpyToSymbol(C_relativeUpperThreshold, &relativeUpperThreshold, sizeof(double));
	
	cudaMemcpyToSymbol(C_min_r, &min_r, sizeof(float));
	cudaMemcpyToSymbol(C_beam_a, &beam_a, sizeof(float));
	cudaMemcpyToSymbol(C_beam_c, &beam_c, sizeof(float));
    
    int threadsPerBlock = 256;
    int point_blocksPerGrid =(point_num + threadsPerBlock - 1) / threadsPerBlock; 
	G_pointsprocess<<<point_blocksPerGrid, threadsPerBlock>>>(dev_mapindex, dev_x, dev_y, dev_z, dev_result, dev_x_ts, dev_y_ts, dev_z_ts, transform, point_num, sensorJacobian, rotationVariance, C_SB_transpose, P_mul_C_BM_transpose, B_r_BS_skew); 
    
	cudaMemcpy(point_var, dev_result, point_num * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(point_z_ts, dev_z_ts, point_num * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(map_index, dev_mapindex, point_num * sizeof(int), cudaMemcpyDeviceToHost);
    cudaMemcpy(point_x_ts, dev_x_ts, point_num * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(point_y_ts, dev_y_ts, point_num * sizeof(float), cudaMemcpyDeviceToHost);
    
    cudaError_t cudaStatus = cudaGetLastError();
    if (cudaStatus != cudaSuccess) 
    {
        fprintf(stderr, "addKernel launch failed: %s\n", cudaGetErrorString(cudaStatus));
        //goto Error;
    }

	cudaFree(dev_x); 	
	cudaFree(dev_y); 
	cudaFree(dev_z); 
	cudaFree(dev_result);
	cudaFree(dev_x_ts); 	
	cudaFree(dev_y_ts); 
    cudaFree(dev_z_ts);
    cudaFree(dev_mapindex); 

	return 0;
}

void Mapvar_update(int length, float var_update)
{
    int cell_num = length * length;
	int threadsPerBlock = 256; 
	int blocksPerGrid =(cell_num + threadsPerBlock - 1) / threadsPerBlock; 
	G_Mapvar_update<<<blocksPerGrid, threadsPerBlock>>>(var_update); 
}

void Fuse(int length, int point_num, int *point_index, int *point_colorR, int *point_colorG, int *point_colorB, float *point_intensity, float *point_height, float *point_var)
{
    int *dev_pointindex;
    float *dev_pointheight;
    float *dev_pointvar;
    float *dev_pointintensity;

    int *dev_pointcolorR;
    int *dev_pointcolorG;
    int *dev_pointcolorB;

    cudaMalloc((void**)&dev_pointcolorR, point_num * sizeof(int)); 
    cudaMalloc((void**)&dev_pointcolorG, point_num * sizeof(int)); 
    cudaMalloc((void**)&dev_pointcolorB, point_num * sizeof(int)); 
    cudaMalloc((void**)&dev_pointindex, point_num * sizeof(int)); 
	cudaMalloc((void**)&dev_pointheight, point_num * sizeof(float)); 
    cudaMalloc((void**)&dev_pointvar, point_num * sizeof(float));
    cudaMalloc((void**)&dev_pointintensity, point_num * sizeof(float));

    cudaMemcpy(dev_pointcolorR, point_colorR, point_num * sizeof(int), cudaMemcpyHostToDevice); 
    cudaMemcpy(dev_pointcolorG, point_colorG, point_num * sizeof(int), cudaMemcpyHostToDevice); 
    cudaMemcpy(dev_pointcolorB, point_colorB, point_num * sizeof(int), cudaMemcpyHostToDevice); 
    cudaMemcpy(dev_pointindex, point_index, point_num * sizeof(int), cudaMemcpyHostToDevice); 
	cudaMemcpy(dev_pointheight, point_height, point_num * sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(dev_pointvar, point_var, point_num * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(dev_pointintensity, point_intensity, point_num * sizeof(float), cudaMemcpyHostToDevice);

    int cell_num = length * length;
	int threadsPerBlock = 256; 
    int blocksPerGrid =(cell_num + threadsPerBlock - 1) / threadsPerBlock; 
    G_fuse<<<blocksPerGrid, threadsPerBlock>>>(dev_pointindex, dev_pointcolorR, dev_pointcolorG, dev_pointcolorB, dev_pointintensity, dev_pointheight, dev_pointvar, point_num); 
    
    cudaFree(dev_pointindex);
    cudaFree(dev_pointheight);
    cudaFree(dev_pointvar);
    cudaFree(dev_pointcolorR);
    cudaFree(dev_pointcolorG);
    cudaFree(dev_pointcolorB);
    cudaFree(dev_pointintensity);
}

__global__ void G_update_mapheight(float height_update)
{    
    int i = blockDim.x * blockIdx.x + threadIdx.x; 
	if (i < Length * Length && map_elevation[i] != -10) {
        map_elevation[i] += height_update;    
    }
  
}
void alignedPosition(float *current_p, float *last_p,  float  resolution, float *opt_alignedPosition)
{
    int indexShift[2];
    float positionShift[2];
    positionShift[0] = current_p[0] - last_p[0];
    positionShift[1] = current_p[1] - last_p[1];
    for (int i = 0; i < 2; i++) {
        indexShift[i] = static_cast<int>(positionShift[i] / resolution + 0.5 * (positionShift[i] > 0 ? 1 : -1));
        opt_alignedPosition[i] = last_p[i] + resolution * indexShift[i];
    }
}

void Map_optmove(float *opt_p, float height_update, float resolution,  int length, float *opt_alignedPosition)
{

    float last_p[2];
    float d_central_coordinate[2];
    cudaMemcpyFromSymbol(&last_p, central_coordinate, sizeof(float) * 2);
    alignedPosition(opt_p, last_p, resolution, opt_alignedPosition);
    d_central_coordinate[0] = opt_alignedPosition[0];
    d_central_coordinate[1] = opt_alignedPosition[1];
    cudaMemcpyToSymbol(central_coordinate, &d_central_coordinate, sizeof(float) * 2);
    int threadsPerBlock = 256; 
    int blocksPerGrid =(length * length + threadsPerBlock - 1) / threadsPerBlock; 

    G_update_mapheight<<<blocksPerGrid, threadsPerBlock>>>(height_update);
    // Debug
    // std::cout << "last:" << last_p[0] << "," << last_p[1] << std::endl;
    // std::cout << "last:" << opt_p[0] << "," << opt_p[1] << std::endl;
    // std::cout << "opt_alignedPosition:" << opt_alignedPosition[0] << "," << opt_alignedPosition[1] << std::endl;
}

void Map_closeloop(float *update_position, float height_update, int length, float resolution)
{
    float h_central_coordinate[2];
    int indexShift[2];
    float positionShift[2];
    float alignedPositionShift[2];
    cudaMemcpyFromSymbol(&h_central_coordinate, central_coordinate, sizeof(float) * 2);
    positionShift[0] = update_position[0] - h_central_coordinate[0];
    positionShift[1] = update_position[1] - h_central_coordinate[1];
    getIndexShiftFromPositionShift(indexShift, positionShift, resolution);
    getPositionShiftFromIndexShift(alignedPositionShift, indexShift, resolution);
    h_central_coordinate[0] = PositionToRange(h_central_coordinate[0], alignedPositionShift[0], resolution);
    h_central_coordinate[1] = PositionToRange(h_central_coordinate[1], alignedPositionShift[1], resolution);
    cudaMemcpyToSymbol(central_coordinate, &h_central_coordinate, sizeof(float) * 2);
    
    int cell_num  = length * length;
    int threadsPerBlock = 256; 
    int blocksPerGrid =(cell_num + threadsPerBlock - 1) / threadsPerBlock; 
    G_update_mapheight<<<blocksPerGrid, threadsPerBlock>>>(height_update);
}

void Map_feature(int length, float *elevation, float *var, int *colorR, int *colorG, int *colorB, float *rough, float *slope, float *traver, float *intensity)
{
    float *d_elevation;
    float *d_var;
    float *d_rough = 0;
    float *d_slope = 0;
    float *d_traver = 0;
    float *d_intensity = 0;
    int *d_colorR;
    int *d_colorG;
    int *d_colorB;

    cudaMalloc((void**)&d_colorR, length * length * sizeof(int)); 
    cudaMalloc((void**)&d_colorG, length * length * sizeof(int)); 
    cudaMalloc((void**)&d_colorB, length * length * sizeof(int)); 
    cudaMalloc((void**)&d_elevation, length * length * sizeof(float)); 
    cudaMalloc((void**)&d_var, length * length * sizeof(float)); 
    cudaMalloc((void**)&d_rough, length * length * sizeof(float)); 
    cudaMalloc((void**)&d_slope, length * length * sizeof(float)); 
    cudaMalloc((void**)&d_traver, length * length * sizeof(float)); 
    cudaMalloc((void**)&d_intensity, length * length * sizeof(float)); 

    int cell_num = length * length;
	int threadsPerBlock = 256; 
	int blocksPerGrid =(cell_num + threadsPerBlock - 1) / threadsPerBlock; 
    G_Mapfeature<<<blocksPerGrid, threadsPerBlock>>>(d_colorR, d_colorG, d_colorB, d_elevation, d_var, d_rough, d_slope, d_traver, d_intensity); 

    cudaMemcpy(colorR, d_colorR, cell_num * sizeof(int), cudaMemcpyDeviceToHost); 
    cudaMemcpy(colorG, d_colorG, cell_num * sizeof(int), cudaMemcpyDeviceToHost); 
    cudaMemcpy(colorB, d_colorB, cell_num * sizeof(int), cudaMemcpyDeviceToHost); 
    cudaMemcpy(elevation, d_elevation, cell_num * sizeof(float), cudaMemcpyDeviceToHost); 
    cudaMemcpy(var, d_var, cell_num * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(rough, d_rough, cell_num * sizeof(float), cudaMemcpyDeviceToHost); 
    cudaMemcpy(slope, d_slope, cell_num * sizeof(float), cudaMemcpyDeviceToHost); 
    cudaMemcpy(traver, d_traver, cell_num * sizeof(float), cudaMemcpyDeviceToHost); 
    cudaMemcpy(intensity, d_intensity, cell_num * sizeof(float), cudaMemcpyDeviceToHost); 

    cudaFree(d_colorR);
    cudaFree(d_colorG);
    cudaFree(d_colorB);
    cudaFree(d_elevation);
    cudaFree(d_var);
    cudaFree(d_rough);
    cudaFree(d_slope);
    cudaFree(d_traver);
    cudaFree(d_intensity);
}

void Raytracing(int length)
{
    int cell_num = length * length;
	int threadsPerBlock = 256; 
    int blocksPerGrid =(cell_num + threadsPerBlock - 1) / threadsPerBlock; 
    
    G_Raytracing<<<blocksPerGrid, threadsPerBlock>>>(); 
    G_Clear_maplowest<<<blocksPerGrid, threadsPerBlock>>>(); 
    cudaError_t cudaStatus = cudaDeviceSynchronize();
    if (cudaStatus != cudaSuccess) 
    {
        fprintf(stderr, "addKernel launch failed: %s\n", cudaGetErrorString(cudaStatus));
        //goto Error;
    }
}
