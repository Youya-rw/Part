/**
***  キャラクタアニメーションのための人体モデルの表現・基本処理 ライブラリ・サンプルプログラム
***  Copyright (c) 2015-, Masaki OSHITA (www.oshita-lab.org)
***  Released under the MIT license http://opensource.org/licenses/mit-license.php
**/

/**
***  逆運動学計算計算（疑似逆行列）アプリケーション
**/

#ifndef  _INVERSE_KINEMATICS_PARTICLE_APP_H_
#define  _INVERSE_KINEMATICS_PARTICLE_APP_H_


// ライブラリ・クラス定義の読み込み
#include "SimpleHuman.h"
#include "SimpleHumanGLUT.h"
#include "InverseKinematicsBaseApp.h"


//
//  逆運動学計算（粒子法）アプリケーションクラス
//
class  InverseKinematicsParticleApp : public InverseKinematicsBaseApp
{
  protected:
	// 逆運動学計算の設定

	// 粒子の位置の初期化フラグ
	bool  init_joint_positions;

	// 逆運動学計算で粒子の位置のみを更新（粒子の位置のみを更新し、姿勢の計算は行わない）
	bool  update_joint_position_only;

  public:
	// コンストラクタ
	InverseKinematicsParticleApp();

	//  開始・リセット
	virtual void  Start();

	// 画面描画
	virtual void  Display();

	// キーボードのキー押下
	virtual void  Keyboard( unsigned char key, int mx, int my );

  public:
	// 逆運動学計算処理

	// 逆運動学計算（粒子法）
	virtual void  ApplyInverseKinematics( Posture & posture, int base_joint_no, int ee_joint_no, const Point3f & ee_joint_position, Matrix3f * ee_joint_orientation = NULL );

	// 関節点の選択・移動のための関節点の位置・向きの更新
	virtual void  UpdateJointPositions( const Posture & posture );
};


// 補助処理（グローバル関数）のプロトタイプ宣言

// 逆運動学計算計算（粒子法）
void  ApplyInverseKinematicsParticle( Posture & posture, int base_joint_no, int ee_joint_no, const Point3f & ee_joint_position );

// 粒子法での全関節点の位置の更新
bool  UpdateJointParticles( const Skeleton * body, const vector< bool > & fixed_joints, vector< Point3f > & joint_positions );

// 全関節の位置から姿勢を計算
// 全関節の位置の配列を入力（joint_positions[関節番号]）
void  ComputePostureFromJointPoints( Posture & pose, const Point3f * joint_positions );

// 指定した関節・体節の位置から姿勢を計算
// 全関節・体節の位置の配列を入力（joint_positions[関節番号], segment_positions[体節番号]）
// 姿勢計算に使用する関節・体節を指定する配列を入力（joint_position_flags[関節番号], segment_position_flags[体節番号]）
void  ComputePostureFromJointPoints( Posture & pose, 
	const Point3f * joint_positions, const Point3f * segment_positions, 
	const bool * joint_position_flags = NULL, const bool * segment_position_flags = NULL );



#endif // _INVERSE_KINEMATICS_PARTICLE_APP_H_
