/**
***  キャラクタアニメーションのための人体モデルの表現・基本処理のサンプルプログラム
***  Copyright (c) 2015-, Masaki OSHITA (www.oshita-lab.org)
**/

/**
***  複数姿勢補間
**/

#ifndef  _MULTIPLE_POSTURE_INTERPOLATION_H_
#define  _MULTIPLE_POSTURE_INTERPOLATION_H_


// ヘッダファイルのインクルード
#include "SimpleHuman.h"


// 複数姿勢補間処理（グローバル関数）のプロトタイプ宣言

// 複数姿勢補間（複数の姿勢を補間）
void  MultiplePostureInterpolation( int num_postures, const Posture ** postures, const float * weights, Posture & p );

// 複数回転補間（複数の回転（四元数）を補間）
void  MultipleRotationInterpolation( int num, const Quat4f * rotations, const float * weights, Quat4f & out_rotation );



#endif // _MULTIPLE_POSTURE_INTERPOLATION_H_
