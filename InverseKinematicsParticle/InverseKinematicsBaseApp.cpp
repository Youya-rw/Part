/**
***  キャラクタアニメーションのための人体モデルの表現・基本処理 ライブラリ・サンプルプログラム
***  Copyright (c) 2015-, Masaki OSHITA (www.oshita-lab.org)
***  Released under the MIT license http://opensource.org/licenses/mit-license.php
**/

/**
***  逆運動学計算アプリケーション
**/


// ライブラリ・クラス定義の読み込み
#include "SimpleHuman.h"
#include "InverseKinematicsBaseApp.h"
#include "BVH.h"
#include "ObjectLayout.h"



//
//  コンストラクタ
//
InverseKinematicsBaseApp::InverseKinematicsBaseApp()
{
	app_name = "Inverse Kinematics";
	body = NULL;
	curr_posture = NULL;

//	ik_control = IK_EE_2D;
	ik_control = IK_EE_3D;

	base_joint_no = -1;
	ee_joint_no = -1;

	layout = new ObjectLayout();
	draw_joints = true;
}


//
//  デストラクタ
//
InverseKinematicsBaseApp::~InverseKinematicsBaseApp()
{
	if ( layout )
		delete  layout;
	if ( curr_posture )
		delete  curr_posture;
	if ( body )
		delete  body;
}



///////////////////////////////////////////////////////////////////////////////
//
//  初期化・設定処理
//


//
//  骨格モデルの設定と姿勢の初期化
//
void  InverseKinematicsBaseApp::InitBodyAndPosture( const Skeleton * new_body )
{
	if ( ( !new_body ) || ( new_body == body ) )
		return;

	body = new_body;

	if ( curr_posture )
		delete  curr_posture;

	curr_posture = new Posture();

	InitPosture( *curr_posture, body );
}


//
//  末端関節の操作方法の設定
//
void  InverseKinematicsBaseApp::SetIKEEControlMethod( IKEEControlType method )
{
	ik_control = method;
}



///////////////////////////////////////////////////////////////////////////////
//
//  イベント処理
//


//
//  初期化
//
void  InverseKinematicsBaseApp::Initialize()
{
	GLUTBaseApp::Initialize();

	// 骨格モデルの初期化に使用する BVHファイル
	const char *  file_name = "sample_walking1.bvh";

	// 動作データを読み込み
	BVH *  bvh = new BVH( file_name );

	// BVH動作から骨格モデルを生成
	if ( bvh->IsLoadSuccess() )
	{
		Skeleton *  new_body = CoustructBVHSkeleton( bvh );

/*		// 姿勢の初期化
		if ( new_body )
		{
			body = new_body;
			curr_posture = new Posture();
			InitPosture( *curr_posture, body );
		}
*/
		// 骨格モデルの設定と姿勢の初期化
		InitBodyAndPosture( new_body );
	}

	// 動作データを削除
	delete  bvh;
}


//
//  開始・リセット
//
void  InverseKinematicsBaseApp::Start()
{
	GLUTBaseApp::Start();

	if ( !curr_posture )
		return;

	// 姿勢初期化
	InitPosture( *curr_posture );

	// 関節点の更新
	UpdateJointPositions( *curr_posture );

	// 支点・末端関節の初期化
	base_joint_no = -1;
	ee_joint_no = -1;

	// 配置操作の選択オブジェクトをクリア
	layout->OnMouseDown( 0, 0 );
}


//
//  画面描画
//
void  InverseKinematicsBaseApp::Display()
{
	GLUTBaseApp::Display();

	// キャラクタを描画
	if ( curr_posture )
	{
		glColor3f( 1.0f, 1.0f, 1.0f );
		DrawPosture( *curr_posture );
		DrawPostureShadow( *curr_posture, shadow_dir, shadow_color );
	}

	// 視点が更新されたら関節点の位置を更新
	if ( curr_posture && is_view_updated )
	{
		UpdateJointPositions( *curr_posture );

		// 視点の更新フラグをクリア
		is_view_updated = false;
	}

	// 関節点を描画
	if ( curr_posture && draw_joints )
	{
		if ( ik_control == IK_EE_2D )
			DrawJoint();
		if ( ik_control == IK_EE_3D )
			DrawJoint3D();
	}
}


//
//  マウスクリック
//
void  InverseKinematicsBaseApp::MouseClick( int button, int state, int mx, int my )
{
	GLUTBaseApp::MouseClick( button, state, mx, my );

	// 左ボタンが押されたら、IKの支点・末端関節を選択
	if ( ( button == GLUT_LEFT_BUTTON ) && ( state == GLUT_DOWN ) )
	{
		// ２次元平面上での選択部位の位置操作
		if ( ik_control == IK_EE_2D )
		{
			// Shiftキーが押されていれば、支点関節を選択
			if ( glutGetModifiers() & GLUT_ACTIVE_SHIFT )
				SelectJoint( mx, my, false );
			// Shiftキーが押されていなければ、末端関節を選択
			else
				SelectJoint( mx, my, true );
		}

		// ３次元空間内での選択部位の位置・向き操作
		if ( ik_control == IK_EE_3D )
		{
			// Shiftキーが押されていれば、支点関節を選択
			if ( glutGetModifiers() & GLUT_ACTIVE_SHIFT )
				SelectJoint3D( mx, my, false );
			// Shiftキーが押されていなければ、末端関節を選択
			else
				SelectJoint3D( mx, my, true );
		}
	}

	if ( ( button == GLUT_LEFT_BUTTON ) && ( state == GLUT_UP ) )
	{
		// ３次元空間内での選択部位の位置・向き操作
		if ( ik_control == IK_EE_3D )
		{
			if ( layout )
				layout->OnMouseUp( mx, my );
		}
	}
}


//
//  マウスドラッグ
//
void  InverseKinematicsBaseApp::MouseDrag( int mx, int my )
{
	// 左ボタンのドラッグ中は、IKの末端関節の目標位置を操作
	if ( drag_mouse_l )
	{
		// ２次元平面上での選択部位の位置操作
		if ( ik_control == IK_EE_2D )
			MoveJoint( mx - last_mouse_x, my - last_mouse_y );

		// ３次元空間内での選択部位の位置・向き操作
		if ( ik_control == IK_EE_3D )
			MoveJoint3D( mx, my );
	}

	GLUTBaseApp::MouseDrag( mx, my );
}


//
//  マウス移動
//
void  InverseKinematicsBaseApp::MouseMotion( int mx, int my )
{
	// ３次元空間内での選択部位の位置・向き操作
	if ( ik_control == IK_EE_3D )
	{
		if ( layout )
			layout->OnMoveMouse( mx, my );
	}

	GLUTBaseApp::MouseMotion( mx, my );
}


//
//  キーボードのキー押下
//
void  InverseKinematicsBaseApp::Keyboard( unsigned char key, int mx, int my )
{
	GLUTBaseApp::Keyboard( key, mx, my );

	// e キーで末端関節の操作方法を変更
	if ( key == 'e' )
	{
		int  change_control = (IKEEControlType)( ( ik_control + 1 ) % NUM_IK_EE_CONTROLS );
		SetIKEEControlMethod( (IKEEControlType) change_control );

		UpdateJointPositions( *curr_posture );
	}

	// v キーで関節点の描画の有無を変更
	if ( key == 'v' )
		draw_joints = !draw_joints;

	// r キーで姿勢をリセット
	if ( key == 'r' )
		Start();
}



///////////////////////////////////////////////////////////////////////////////
//
//  逆運動学計算処理
//


//
//  逆運動学計算
//  入出力姿勢、支点関節番号（-1の場合はルートを支点とする）、末端関節番号、末端関節の目標位置、末端関節の目標向き（NUUのときは省略）を指定
//
void  InverseKinematicsBaseApp::ApplyInverseKinematics( 
	Posture & posture, int base_joint_no, int ee_joint_no, const Point3f & ee_joint_position, Matrix3f * ee_joint_orientation )
{
}


//
//  関節点の選択・移動のための関節点の位置・向きの更新
//
void  InverseKinematicsBaseApp::UpdateJointPositions( const Posture & posture )
{
	// ２次元平面上での選択部位の位置操作
	if ( ik_control == IK_EE_2D )
		UpdateJointPositions2D( posture );

	// ３次元空間内での選択部位の位置・向き操作
	if ( ik_control == IK_EE_3D )
		UpdateJointPositions3D( posture );
}



///////////////////////////////////////////////////////////////////////////////
//
//  ２次元平面上での関節点の選択・移動のための補助処理
//


//
//  関節点の位置の更新
//
void  InverseKinematicsBaseApp::UpdateJointPositions2D( const Posture & posture )
{
	if ( !curr_posture )
		return;

	// 順運動学計算
	vector< Matrix4f >  seg_frame_array;
	ForwardKinematics( posture, seg_frame_array, joint_world_positions );

	// OpenGL の変換行列を取得
	double  model_view_matrix[ 16 ];
	double  projection_matrix[ 16 ];
	int  viewport_param[ 4 ];
	glGetDoublev( GL_MODELVIEW_MATRIX, model_view_matrix );
	glGetDoublev( GL_PROJECTION_MATRIX, projection_matrix );
	glGetIntegerv( GL_VIEWPORT, viewport_param );

	// 画面上の各関節点の位置を計算
	int  num_joints = joint_world_positions.size();
	GLdouble  spx, spy, spz;
	joint_screen_positions.resize( num_joints );
	for ( int i = 0; i < num_joints; i++ )
	{
		const Point3f &  wp = joint_world_positions[ i ];
		Point3f &  sp = joint_screen_positions[ i ];

		gluProject( wp.x, wp.y, wp.z,
			model_view_matrix, projection_matrix, viewport_param,
			&spx, &spy, &spz );
		sp.x = spx;
		sp.y = viewport_param[ 3 ] - spy;
	}

	// 関節点の向きを取得（末端側の体節の向きを関節の向きとして取得）
	joint_world_orientations.resize( posture.body->num_joints );
	for ( int i = 0; i < posture.body->num_joints; i++ )
	{
		seg_frame_array[ posture.body->joints[ i ]->segments[ 1 ]->index ].get( &joint_world_orientations[ i ] );
	}
}


//
//  関節点の選択
//
void  InverseKinematicsBaseApp::SelectJoint( int mouse_x, int mouse_y, bool ee_or_base )
{
	if ( !curr_posture )
		return;

	const float  distance_threthold = 20.0f;
	float  distance, min_distance = -1.0f;
	int  closesed_joint_no = -1;
	float  dx, dy;

	// 入力座標と最も近い位置にある関節を探索
	for ( int i=0; i<joint_screen_positions.size(); i++ )
	{
		dx = joint_screen_positions[ i ].x - mouse_x;
		dy = joint_screen_positions[ i ].y - mouse_y;
		distance = sqrt( dx * dx + dy * dy );
		if ( ( i == 0 ) || ( distance <= min_distance ) )
		{
			min_distance = distance;
			closesed_joint_no = i;
		}
	}

	// 距離が閾値以下であれば選択
	if ( ee_or_base )
	{
		if ( min_distance < distance_threthold )
			ee_joint_no = closesed_joint_no;
		else
			ee_joint_no = -1;
	}
	else
	{
		if ( min_distance < distance_threthold )
			base_joint_no = closesed_joint_no;
		else
			base_joint_no = -1;
	}
}


//
//  関節点の移動（視線に垂直な平面上で上下左右に移動する）
//
void  InverseKinematicsBaseApp::MoveJoint( int mouse_dx, int mouse_dy )
{
	if ( !curr_posture )
		return;

	// 末端関節が選択されていなければ終了
	if ( ee_joint_no == -1 )
		return;

	// 画面上の移動量と３次元空間での移動量の比率
	const float  mouse_pos_scale = 0.01f;

	// OpenGL の変換行列を取得
	double  model_view_matrix[ 16 ];
	glGetDoublev( GL_MODELVIEW_MATRIX, model_view_matrix );

	Vector3f  vec;
	Point3f &  pos = joint_world_positions[ ee_joint_no ];

	// カメラ座標系のX軸方向に移動
	vec.set( model_view_matrix[ 0 ], model_view_matrix[ 4 ], model_view_matrix[ 8 ] );
	pos.scaleAdd( mouse_dx * mouse_pos_scale, vec, pos );

	// カメラ座標系のY軸方向に移動
	vec.set( model_view_matrix[ 1 ], model_view_matrix[ 5 ], model_view_matrix[ 9 ] );
	pos.scaleAdd( - mouse_dy * mouse_pos_scale, vec, pos );

	// 逆運動学計算を適用（末端関節の目標位置のみを指定、目標向きは指定しない）
	ApplyInverseKinematics( *curr_posture, base_joint_no, ee_joint_no, pos, &joint_world_orientations[ ee_joint_no ] );

	// 関節点の更新
	UpdateJointPositions( *curr_posture );
}


//
//  関節点の描画
//
void  InverseKinematicsBaseApp::DrawJoint()
{
	if ( !curr_posture )
		return;

	// デプステストを無効にして、前面に上書きする
	glDisable( GL_DEPTH_TEST );

	// 関節点を描画（球を描画）
	for ( int i=0; i<joint_world_positions.size(); i++ )
	{
		// 支点関節は赤で描画
		if ( i == base_joint_no )
			glColor3f( 1.0f, 0.0f, 0.0f );
		// 末端関節は緑で描画
		else if ( i == ee_joint_no )
			glColor3f( 0.0f, 1.0f, 0.0f );
		// 他の関節は青で描画
		else
			glColor3f( 0.0f, 0.0f, 1.0f );

		// 関節位置に球を描画
		const Point3f &  pos = joint_world_positions[ i ];
		glPushMatrix();
			glTranslatef( pos.x, pos.y, pos.z );
			glutSolidSphere( 0.025f, 16, 16 );
		glPopMatrix();
	}

	// 支点関節が指定されていない場合は、ルート体節を支点とする（ルート体節の位置に球を描画）
	if ( base_joint_no == -1 )
	{
		// ルート体節位置に球を描画
		glColor3f( 1.0f, 0.0f, 0.0f );
		const Point3f &  pos = curr_posture->root_pos;
		glPushMatrix();
			glTranslatef( pos.x, pos.y, pos.z );
			glutSolidSphere( 0.025f, 16, 16 );
		glPopMatrix();
	}

	glEnable( GL_DEPTH_TEST );
}



///////////////////////////////////////////////////////////////////////////////
//
//  ３次元空間内での関節点の選択・移動のための補助処理
//


//
//  関節点の位置の更新
//
void  InverseKinematicsBaseApp::UpdateJointPositions3D( const Posture & posture )
{
	if ( !layout )
		return;

	// 順運動学計算
	vector< Matrix4f >  seg_frame_array;
	ForwardKinematics( posture, seg_frame_array, joint_world_positions );

	const Skeleton *  body = posture.body;
	int  no;
	Point3f  pos;
	Matrix3f  ori;

	// オブジェクトの登録を初期化（全関節点＋腰）
	int  num_objects = posture.body->num_joints + 1;
	if ( layout->GetNumObjects() != num_objects )
	{
		layout->DeleteAllObjects();

		for ( int i = 0; i < num_objects; i++ )
		{
			no = layout->AddObject();
			layout->SetObjectSize( no, 0.2f );
		}
	}

	// オブジェクトの位置・向きを更新
	for ( int i = 0; i < num_objects; i++ )
	{
		// 関節点の位置・向きを取得
		if ( i < posture.body->num_joints )
		{
			pos = joint_world_positions[ i ];
			seg_frame_array[ body->joints[ i ]->segments[ 1 ]->index ].get( &ori );
		}
		// 腰の位置・向きを取得
		else
		{
			pos = posture.root_pos;
			ori = posture.root_ori;
		}

		// オブジェクトの位置・向きを設定
		layout->SetObjectPos( i, pos );
		layout->SetObjectOri( i, ori );
	}

	// オブジェクト位置・視点の変更を通知
	layout->Update();
}


//
//  関節点の選択
//
void  InverseKinematicsBaseApp::SelectJoint3D( int mouse_x, int mouse_y, bool ee_or_base )
{
	if ( !layout )
		return;

	// 現在の選択オブジェクトを取得
	int  no = layout->GetCurrentObject();

	// 配置操作の処理呼び出し
	layout->OnMouseDown( mouse_x, mouse_y );

	// 新しいオブジェクトが選択されたら、支点・末端関節として設定
	if ( layout->GetCurrentObject() != no )
	{
		no = layout->GetCurrentObject();

		if ( no == layout->GetNumObjects() - 1 )
			no = -1;

		if ( no == -1 )
			ee_joint_no = -1;
		else if ( ee_or_base )
			ee_joint_no = no;
		else
			base_joint_no = no;
	}
}


//
//  関節点の移動（視線に垂直な平面上で上下左右に移動する）
//
void  InverseKinematicsBaseApp::MoveJoint3D( int mouse_x, int mouse_y )
{
	if ( !layout )
		return;

	// 配置操作の処理呼び出し
	layout->OnMoveMouse( mouse_x, mouse_y );

	// 操作中のオブジェクトを取得（オブジェクトが選択されていなければ終了）
	int  no = layout->GetCurrentObject();
	if ( no == -1 )
		return;

	// 操作中のオブジェクトの位置・向きを末端関節の目標位置・向きとして取得
	Point3f  pos;
	Matrix3f  ori;
	pos = layout->GetPosition( no );
	ori = layout->GetOrientation( no );

	// 逆運動学計算を適用（末端関節の目標位置と目標向きを指定）
	ApplyInverseKinematics( *curr_posture, base_joint_no, ee_joint_no, pos, &ori );

	// 関節点の更新
	UpdateJointPositions( *curr_posture );
}


//
//  関節点の描画
//
void  InverseKinematicsBaseApp::DrawJoint3D()
{
	DrawJoint();

	if ( !layout )
		return;

	layout->Render();
}



