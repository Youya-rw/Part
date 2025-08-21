/**
***  キャラクタアニメーションのための人体モデルの表現・基本処理 ライブラリ・サンプルプログラム
***  Copyright (c) 2015-, Masaki OSHITA (www.oshita-lab.org)
***  Released under the MIT license http://opensource.org/licenses/mit-license.php
**/

/**
***  逆運動学計算計算アプリケーション
**/

#ifndef  _INVERSE_KINEMATICS_BASE_APP_H_
#define  _INVERSE_KINEMATICS_BASE_APP_H_


// ライブラリ・クラス定義の読み込み
#include "SimpleHuman.h"
#include "SimpleHumanGLUT.h"

class  ObjectLayout;


//
//  逆運動学計算アプリケーション基底クラス
//
class  InverseKinematicsBaseApp : public GLUTBaseApp
{
  public:
	// 列挙型の定義

	// 末端関節の操作方法
	enum  IKEEControlType
	{
		IK_EE_2D,      // ２次元平面上での選択部位の位置操作
		IK_EE_3D,      // ３次元空間内での選択部位の位置・向き操作
		NUM_IK_EE_CONTROLS
	};

  protected:
	// キャラクタ情報

	// キャラクタの骨格
	const Skeleton *  body;

	// キャラクタの姿勢
	Posture *  curr_posture;

  protected:
	// 逆運動学計算の設定

	// 支点・末端関節番号
	int  base_joint_no;
	int  ee_joint_no;

  protected:
	// 末端関節の操作のための変数

	// 末端関節の操作方法
	IKEEControlType  ik_control;

	// 関節点の位置・向き
	vector< Point3f >  joint_world_positions;
	vector< Point3f >  joint_screen_positions;
	vector< Matrix3f >  joint_world_orientations;

	// 配置操作
	ObjectLayout *  layout;

	// 関節点の描画の有無の設定
	bool  draw_joints;


  public:
	// コンストラクタ
	InverseKinematicsBaseApp();

	// デストラクタ
	virtual ~InverseKinematicsBaseApp();

  public:
	// 初期化・設定処理

	// 骨格モデルの設定と姿勢の初期化
	void  InitBodyAndPosture( const Skeleton * new_body );

	// 末端関節の操作方法の設定
	void  SetIKEEControlMethod( IKEEControlType method );

  public:
	// イベント処理

	//  初期化
	virtual void  Initialize();

	//  開始・リセット
	virtual void  Start();

	//  画面描画
	virtual void  Display();

	// マウスクリック
	virtual void  MouseClick( int button, int state, int mx, int my );

	//  マウスドラッグ
	virtual void  MouseDrag( int mx, int my );

	// マウス移動
	virtual void  MouseMotion( int mx, int my );

	// キーボードのキー押下
	virtual void  Keyboard( unsigned char key, int mx, int my );

  public:
	// 逆運動学計算処理

	// 逆運動学計算
	virtual void  ApplyInverseKinematics( Posture & posture, 
		int base_joint_no, int ee_joint_no, const Point3f & ee_joint_position, Matrix3f * ee_joint_orientation = NULL );

	// 関節点の選択・移動のための関節点の位置・向きの更新
	virtual void  UpdateJointPositions( const Posture & posture );

  protected:
	// ２次元平面上での関節点の選択・移動のための補助処理

	// 関節点の位置の更新
	void  UpdateJointPositions2D( const Posture & posture );

	// 関節点の選択
	void  SelectJoint( int mouse_x, int mouse_y, bool ee_or_base );

	// 関節点の移動（視線に垂直な平面上で上下左右に移動する）
	void  MoveJoint( int mouse_dx, int mouse_dy );

	// 関節点の描画
	void  DrawJoint();

  protected:
	// ３次元空間内での関節点の選択・移動のための補助処理

	// 関節点の位置の更新
	void  UpdateJointPositions3D( const Posture & posture );

	// 関節点の選択
	void  SelectJoint3D( int mouse_x, int mouse_y, bool ee_or_base );

	// 関節点の移動（視線に垂直な平面上で上下左右に移動する）
	void  MoveJoint3D( int mouse_x, int mouse_y );

	// 関節点の描画
	void  DrawJoint3D();
};



#endif // _INVERSE_KINEMATICS_BASE_APP_H_
