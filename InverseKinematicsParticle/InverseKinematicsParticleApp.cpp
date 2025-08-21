/**
***  キャラクタアニメーションのための人体モデルの表現・基本処理 ライブラリ・サンプルプログラム
***  Copyright (c) 2015-, Masaki OSHITA (www.oshita-lab.org)
***  Released under the MIT license http://opensource.org/licenses/mit-license.php
**/

/**
***  逆運動学計算計算（疑似逆行列）アプリケーション
**/


// ライブラリ・クラス定義の読み込み
#include "SimpleHuman.h"
#include "InverseKinematicsParticleApp.h"
#include "ObjectLayout.h"

// 複数回転補間関数の定義の読み込み
#include "MultiplePostureInterpolation.h"

// 標準算術関数・定数の定義
#define  _USE_MATH_DEFINES
#include <math.h>



//
//  コンストラクタ
//
InverseKinematicsParticleApp::InverseKinematicsParticleApp() : InverseKinematicsBaseApp()
{
	app_name = "Inverse Kinematics (Particle)";

	ik_control = IK_EE_2D;

	// 逆運動学計算で粒子の位置のみを更新（粒子の位置のみを更新し、姿勢の計算は行わない）
	update_joint_position_only = false;
}


//
//  開始・リセット
//
void  InverseKinematicsParticleApp::Start()
{
	// 粒子の位置の初期化フラグを設定
	init_joint_positions = true;

	// 基底クラスの処理を呼び出し
	InverseKinematicsBaseApp::Start();
}


//
//  画面描画
//
void  InverseKinematicsParticleApp::Display()
{
	// 基底クラスの処理を呼び出し（キャラクタ・関節点の描画）
	InverseKinematicsBaseApp::Display();

	// 現在のモードを表示
	DrawTextInformation( 0, "Inverse Kinematics (Particle)" );

	// 末端関節の操作方法を表示
	if ( ik_control == IK_EE_2D )
		DrawTextInformation( 2, "Control: 2D" );
	else if ( ik_control == IK_EE_3D )
		DrawTextInformation( 2, "Control: 3D" );
}


//
//  キーボードのキー押下
//
void  InverseKinematicsParticleApp::Keyboard( unsigned char key, int mx, int my )
{
	// 基底クラスの処理を呼び出し
	InverseKinematicsBaseApp::Keyboard( key, mx, my );

	// x キーで粒子の位置のみの更新の設定を変更
	if ( key == 'x' )
	{
		update_joint_position_only = !update_joint_position_only;
	}
}


//
//  逆運動学計算（粒子法）
//  入出力姿勢、支点関節番号（-1の場合はルートを支点とする）、末端関節番号、末端関節の目標位置、末端関節の目標向き（NUUのときは省略）を指定
//
void  InverseKinematicsParticleApp::ApplyInverseKinematics( 
	Posture & posture, int base_joint_no, int ee_joint_no, const Point3f & ee_joint_position, Matrix3f * ee_joint_orientation )
{
	// 逆運動学計算（粒子法）を適用
	if ( !update_joint_position_only )
		ApplyInverseKinematicsParticle( posture, base_joint_no, ee_joint_no, ee_joint_position );

	// 粒子の移動の動作確認用の処理（テスト用）
	//（本来の処理を以下の処理に置き換えることで、粒子の移動のみの動作確認）
	if ( update_joint_position_only )
	{
		// 骨格情報の取得
		int  num_segments = posture.body->num_segments;
		int  num_joints = posture.body->num_joints;

		// 計算用変数
		vector< Point3f >  joint_positions( num_joints );
		vector< bool >  fixed_joints( num_joints, false );

		// 引数チェック
		if ( !posture.body || ( ee_joint_no == -1 ) || ( base_joint_no == ee_joint_no ) )
			return;

		for ( int i = 0; i < num_joints; i++ )
			joint_positions[ i ] = joint_world_positions[ i ];

		// 目標関節の位置を設定
		joint_positions[ ee_joint_no ] = ee_joint_position;

		// 固定する関節（末端関節と支点関節）を設定
		if ( ( ee_joint_no >= 0 ) && ( ee_joint_no < num_joints ) )
			fixed_joints[ ee_joint_no ] = true;
		if ( ( base_joint_no >= 0 ) && ( base_joint_no < num_joints ) )
			fixed_joints[ base_joint_no ] = true;

		// 関節点の位置を更新
		for ( int i = 0; i< 10; i++ )
		{
			// 全関節点の位置の更新の繰り返し計算
			if ( UpdateJointParticles( posture.body, fixed_joints, joint_positions ) )
				break;
		}

		// 関節点の位置を記録
		for ( int i = 0; i < num_joints; i++ )
			joint_world_positions[ i ] = joint_positions[ i ];
	}
}


//
//  関節点の選択・移動のための関節点の位置・向きの更新
//
void  InverseKinematicsParticleApp::UpdateJointPositions( const Posture & posture )
{
	// 基底クラスの処理を呼び出し
	if ( !update_joint_position_only )
		return  InverseKinematicsBaseApp::UpdateJointPositions( posture );

	// 粒子の移動の動作確認用の処理（テスト用）
	//（本来の処理を以下の処理に置き換えることで、粒子の移動のみの動作確認）
	if ( update_joint_position_only )
	{
		// 初期姿勢にもとづいて関節点を更新（最初に一度だけ実行）
		if ( init_joint_positions )
		{
			init_joint_positions = false;
			return  InverseKinematicsBaseApp::UpdateJointPositions( posture );
		}

		// ３次元空間での関節点の操作のための関節点の位置の更新
		if ( ik_control == IK_EE_3D )
		{
			Point3f  pos;
			int  num_objects = posture.body->num_joints + 1;

			// オブジェクトの位置・向きを更新
			for ( int i = 0; i < num_objects; i++ )
			{
				// 関節点の位置を取得
				if ( i < posture.body->num_joints )
					pos = joint_world_positions[ i ];
				// 腰の位置を取得
				else
					pos = posture.root_pos;

				// オブジェクトの位置を設定
				layout->SetObjectPos( i, pos );
			}
		}
	}
}



///////////////////////////////////////////////////////////////////////////////
//
//  逆運動学計算（粒子法）（グローバル関数）
//


// 逆運動学計算（粒子法）の計算のためのパラメタ

// 最大繰り返し数の設定
static const int  max_iteration = 10;

// 位置が収束したと判断するための閾値の設定
static const float  distance_threshold = 0.001f;

// 関節点に加える力の関節点間の距離のずれに対する比率
static const float  force_scale = 0.1f;


// 逆運動学計算（粒子法）の内部処理のための関数のプロトタイプ宣言

// 姿勢計算の繰り返し計算（全関節のワールド座標系での位置から計算）
void  ComputePostureFromJointPointsIteration( Posture & pose, int joint_no, 
	const Point3f * joint_positions, const bool * joint_position_flags, 
	const Point3f * segment_positions, const bool * segment_position_flags );

// ２つのベクトルにもとづく回転を計算（src_vec の方向を dest_vec に向けるための２自由度の回転を計算）
void  ComputeVectorRotation( const Vector3f & src_vec, const Vector3f & dest_vec, Matrix3f & rot );

// ２つのベクトルにもとづく回転を計算（src_vec の方向を dest_vec に向けるための２自由度の回転を計算）
void  ComputeVectorRotation( const Vector3f & src_vec, const Vector3f & dest_vec, AxisAngle4f & rot_vec_angle );

// ３組みの２つのベクトルにもとづく回転を計算（src_vec の方向を dest_vec に向けるための３自由度の回転を計算）
bool  ComputeTripleVectorRotation( const Vector3f src_vec[ 3 ], const Vector3f dest_vec[ 3 ], Matrix3f & rot );

// ｎ組みの２つのベクトルにもとづく回転を計算（src_vec の方向を dest_vec に向けるための３自由度の回転を計算）
bool  ComputeMultipleVectorRotation( int num_vectors, const Vector3f * src_vec, const Vector3f * dest_vec, Matrix3f & rot );



//
//  逆運動学計算計算（粒子法）
//
void  ApplyInverseKinematicsParticle( Posture & posture, int base_joint_no, int ee_joint_no, const Point3f & ee_joint_position )
{
	// 骨格情報の取得
	int  num_segments = posture.body->num_segments;
	int  num_joints = posture.body->num_joints;

	// 計算用変数
	vector< Matrix4f >  segment_frames( num_segments );
	vector< Point3f >  joint_positions( num_joints );
	vector< bool >  fixed_joints( num_joints, false );

	// 引数チェック
	if ( !posture.body || ( ee_joint_no == -1 ) || ( base_joint_no == ee_joint_no ) )
		return;

	// 順運動学（FK）計算
	ForwardKinematics( posture, segment_frames, joint_positions );

	// 目標関節の位置を設定
	joint_positions[ ee_joint_no ] = ee_joint_position;

	// 固定する関節（末端関節と支点関節）を設定
	if ( ( ee_joint_no >= 0 ) && ( ee_joint_no < num_joints ) )
		fixed_joints[ ee_joint_no ] = true;
	if ( ( base_joint_no >= 0 ) && ( base_joint_no < num_joints ) )
		fixed_joints[ base_joint_no ] = true;

	// 関節点の位置を更新
	for ( int i = 0; i< max_iteration; i++ )
	{
		// 全関節点の位置の更新の繰り返し計算
		if ( UpdateJointParticles( posture.body, fixed_joints, joint_positions ) )
			break;
	}

	// 全関節の位置から姿勢を計算
	ComputePostureFromJointPoints( posture, &joint_positions.front() );
}



///////////////////////////////////////////////////////////////////////////////
//
//  逆運動学計算（粒子法）の内部処理（グローバル関数）
//


//
//  粒子法での全関節点の位置の更新
//
bool  UpdateJointParticles( 
	const Skeleton * body, const vector< bool > & fixed_joints, vector< Point3f > & joint_positions )
{
	// 計算用変数
	int  joint0, joint1;
	Vector3f  vec, force;
	float  dist = 0.0f, dist_org = 0.0f;

	// 収束判定のための閾値
	const float distance_threshold = 0.001f; // 距離の収束閾値
	const float force_scale = 0.1f;         // 力の係数
	const float convergence_threshold_for_forces_sq = 0.0001f; // 力の合計の二乗の収束閾値

	// 骨格情報の取得
	int  num_segments = body->num_segments;
	int  num_joints = body->num_joints;

	// 全関節点に働く力の配列を初期化
	vector< Point3f >  joint_forces( num_joints );
	for ( int i = 0; i < num_joints; i++ )
		joint_forces[ i ].set( 0.0f, 0.0f, 0.0f );


	// ※ レポート課題
	
	// 全関節点に働く力を更新
	for ( int i = 0; i < num_segments; i++ )
	{
		// 隣接する関節の組み合わせに対して繰り返し
		const Segment *  seg = body->segments[ i ];
		for ( int j = 0; j < seg->num_joints - 1; j++ )
		{
			for ( int k = j + 1; k < seg->num_joints; k++ )
			{
				// 体節に隣接する２つの関節を取得
				joint0 = seg->joints[ j ]->index;
				joint1 = seg->joints[ k ]->index;

				// 関節点間の距離を計算
				vec.sub(joint_positions[joint1], joint_positions[joint0]);
				dist = vec.length();

				// 元の関節間の距離を取得
				// Segmentクラスのjoint_positionsメンバを使用して、体節内の関節の初期位置間の距離を計算
				Vector3f original_vec_in_segment;
				original_vec_in_segment.sub(seg->joint_positions[k], seg->joint_positions[j]);
				dist_org = original_vec_in_segment.length();

				// 距離が収束していれば力は加えない（距離の閾値 distance_threshold を使用）
				if ( fabs(dist - dist_org) < distance_threshold )
					continue;

				// 力の計算
				force = vec;
				force.normalize();
				force *= (dist - dist_org) * force_scale;

				// 関節点に力を適用
				joint_forces[ joint0 ].add( force );
				joint_forces[ joint1 ].sub( force );
			}
		}
	}

	// 全ての関節点間の距離が収束していれば、繰り返し処理を終了
	// ここでは、全ての関節点に働く力の合計が非常に小さいかどうかで判断する
	float total_force_magnitude_sq = 0.0f;
	for (int i = 0; i < num_joints; i++) {
	    total_force_magnitude_sq += joint_forces[i].x * joint_forces[i].x + joint_forces[i].y * joint_forces[i].y + joint_forces[i].z * joint_forces[i].z;
	}
	if (total_force_magnitude_sq < convergence_threshold_for_forces_sq) // convergence_threshold_for_forces_sqはグローバル変数として定義されていると仮定
		return  true;

	// 全関節点の位置を更新
	for ( int i = 0; i < num_joints; i++ )
	{
		// 固定された関節は更新しない
		if (fixed_joints[i])
			continue;

		// 力を位置に適用（ここでは単純に力をそのまま位置に加えるが、実際にはタイムステップや質量などを考慮する）
		joint_positions[i].add(joint_forces[i]);
	}


	return  false;
}


//
//  全関節の位置から姿勢を計算
//  全関節・体節の位置の配列を入力（joint_positions[関節番号], segment_positions[体節番号]）
//  姿勢計算に使用する関節・体節を指定する配列を入力（joint_position_flags[関節番号], segment_position_flags[体節番号]）
//
void  ComputePostureFromJointPoints( Posture & pose, const Point3f * joint_positions )
{
	// 指定した関節・体節の位置から姿勢を計算（全関節の位置の情報のみを指定）
	ComputePostureFromJointPoints( pose, joint_positions, NULL, NULL, NULL );
}


//
//  指定した関節・体節の位置から姿勢を計算
//  全関節・体節の位置の配列を入力（joint_positions[関節番号], segment_positions[体節番号]）
//  姿勢計算に使用する関節・体節を指定する配列を入力（joint_position_flags[関節番号], segment_position_flags[体節番号]）
//
void  ComputePostureFromJointPoints( Posture & pose, 
	const Point3f * joint_positions, const Point3f * segment_positions, 
	const bool * joint_position_flags, const bool * segment_position_flags )
{
	Joint *  joint, * next_joint;
	const Segment *  segment;

	Vector3f  src_vec[ 3 ], dest_vec[ 3 ];
	Vector3f  root_vec;
	Point3f   root_pos;
	Matrix3f  root_rot;

	// 各関節の回転が計算済みかどうかのフラグを初期化
	int  num_joint = pose.body->num_joints;
	
	// 引数チェック
	if ( !pose.body || !joint_positions )
		return;

	Point3f  src_center, dest_center;

	// 腰の中心位置を計算（腰に接続される全関節の中心位置を計算）
	segment = pose.body->segments[ 0 ];
	dest_center.set( 0.0f, 0.0f, 0.0f );
	for ( int i = 0; i < segment->num_joints; i++ )
		dest_center.add( joint_positions[ segment->joints[ i ]->index ] );
	dest_center.scale( 1.0f / (float) segment->num_joints );

	// 腰の体節の中心位置を計算
	src_center.set( 0.0f, 0.0f, 0.0f );
	for ( int i = 0; i < segment->num_joints; i++ )
		src_center.add( segment->joint_positions[ i ] );
	src_center.scale( 1.0f / (float) segment->num_joints );

	// 腰の向きを計算（腰の接続関節数が３つの場合のみ計算可能とする）
	if ( segment->num_joints >= 3 )
	{
		// ３つの接続関節の回転前・回転後の位置を取得
		for ( int i = 0; i < 3; i++ )
		{
			// 接続関節を取得
			joint = segment->joints[ i ];

			// 腰の中心から関節への回転前の初期位置を取得（腰のローカル座標系）
			src_vec[ i ].sub( segment->joint_positions[ i ], src_center );

			// 腰の中心から関節への回転後の目標位置を取得（ワールド座標系）
			dest_vec[ i ].sub( joint_positions[ joint->index ], dest_center );
		}

		// 腰の中心から関節へのベクトルの組み合わせが正則行列になるように垂直方向のオフセットを加える
		Vector3f  src_offset, dest_offset;
		src_offset.cross( src_vec[ 1 ] - src_vec[ 0 ], src_vec[ 2 ] - src_vec[ 0 ] );
		dest_offset.cross( dest_vec[ 1 ] - dest_vec[ 0 ], dest_vec[ 2 ] - dest_vec[ 0 ] );
		for ( int i = 0; i < 3; i++ )
		{
			src_vec[ i ].sub( src_offset );
			dest_vec[ i ].sub( dest_offset );
		}

		// ３組みの２つのベクトルにもとづく回転を計算（src_vec の方向を dest_vecに 向けるための３自由度の回転を計算）
		if ( !ComputeTripleVectorRotation( src_vec, dest_vec, root_rot ) )
		{
			// 回転が求まらなかった場合は、
			// ｎ組みの２つのベクトルにもとづく回転を計算（src_vec の方向を dest_vec に向けるための３自由度の回転を計算）
			if ( !ComputeMultipleVectorRotation( 3, src_vec, dest_vec, root_rot ) )
			{
				// 回転が求まらなかった場合は、回転なしとする
				root_rot.setIdentity();
			}
		}
	}

	// 腰の位置を計算
	root_rot.transform( src_center, &root_vec );
	root_pos.sub( dest_center, root_vec );

	// 腰の位置・向きを設定
	pose.root_pos = root_pos;
	pose.root_ori = root_rot;

	// 関節回転を初期化
	for ( int i = 0; i < num_joint; i++ )
		pose.joint_rotations[ i ].setIdentity();

	// 腰の体節を取得
	segment = pose.body->segments[ 0 ];

	// 腰に隣接する全ての関節に対して繰り返し計算を呼び出し
	for ( int i = 0; i < segment->num_joints; i ++ )
	{
		// 隣接する関節を取得
		next_joint = segment->joints[ i ];

		// 隣接する関節に対して再帰呼び出し
		ComputePostureFromJointPointsIteration( pose, next_joint->index, 
			joint_positions, joint_position_flags, segment_positions, segment_position_flags );
	}
}


//
//  姿勢計算の繰り返し計算（全関節のワールド座標系での位置から計算）
//
void  ComputePostureFromJointPointsIteration( Posture & pose, int joint_no, 
	const Point3f * joint_positions, const bool * joint_position_flags, 
	const Point3f * segment_positions, const bool * segment_position_flags )
{
	// 計算用変数
	Joint *  joint, * next_joint;
	Segment *  next_segment, * prev_segment;
	Vector3f  vec;
	Matrix4f  joint_frame;
	Matrix3f  rot_mat;

	// 末端側の関節・体節の現在・目標位置（最大３つ）
	Vector3f  src_vec[ 3 ], dest_vec[ 3 ];

	// 末端側の全関節・体節の現在・目標位置の合計
	Vector3f  sum_src_vec, sum_dest_vec;

	// 末端側の全関節・体節の数
	int  count = 0;

	// 対象関節を取得
	joint = pose.body->joints[ joint_no ];

	// ルート・末端側の隣接体節を取得
	prev_segment = joint->segments[ 0 ];
	next_segment = joint->segments[ 1 ];

	// 順運動学計算
	vector< Matrix4f >  seg_frame_array( pose.body->num_segments );
	vector< Point3f >  joi_pos_array( pose.body->num_joints );
	ForwardKinematics( pose, seg_frame_array, joi_pos_array );

	// ワールド座標系から現在の関節のローカル座標系への変換行列を取得
	joint_frame.setIdentity();
	joint_frame.set(seg_frame_array[prev_segment->index]);

	// 末端側の全関節の位置を平均した現在・目標位置を計算
	for ( int i = 0; i < next_segment->num_joints; i++ )
	{
		// 末端側の関節を取得
		next_joint = next_segment->joints[ i ];
		if ( next_joint->index == joint->index )
			continue;

		// 関節点の位置が存在しない場合はスキップ
		if ( joint_position_flags && !joint_position_flags[ next_joint->index ] )
			continue;

		// 現在の関節から末端側の関節点への回転前の初期ベクトルを計算（現在の関節のローカル座標系）


		// ※ レポート課題
		vec.sub(joi_pos_array[next_joint->index], joi_pos_array[joint_no]);
		joint_frame.transform(&vec);

		// 計算結果のベクトルを記録、ベクトルの和を計算
		src_vec[ count ] = vec;
		sum_src_vec.add( vec );

		// 現在の関節から末端側の関節点への回転後の目標ベクトルを計算
		// ※ レポート課題
		vec.sub(joint_positions[next_joint->index], joi_pos_array[joint_no]);
		joint_frame.transform(&vec);


		// 計算結果のベクトルを記録、ベクトルの和を計算
		dest_vec[ count ] = vec;
		sum_dest_vec.add( vec );

		// 存在する末端側の関節・体節点の数をカウント
		count++;
	}

	// 末端側の体節の位置が存在する場合は、体節の位置も現在・目標位置に加える
	if ( segment_positions && segment_position_flags[ next_segment->index ] && ( count < 3 ) )
	{
		// 現在の関節から末端側の体節点への回転前の初期ベクトルを計算（現在の関節のローカル座標系）
		vec.negate( next_segment->joint_positions[ 0 ] );

		// 計算結果のベクトルを記録、ベクトルの和を計算
		src_vec[ count ] = vec;
		sum_src_vec.add( vec );

		// 現在の関節から末端側の体節点への回転後の目標ベクトルを計算（現在の関節のローカル座標系）
		vec.sub( segment_positions[ next_segment->index ], joi_pos_array[ joint_no ] );
		joint_frame.transform( &vec );

		// 計算結果のベクトルを記録、ベクトルの和を計算
		dest_vec[ count ] = vec;
		sum_dest_vec.add( vec );

		// 存在する末端側の関節・体節点の数をカウント
		count++;
	}

	// 末端側の３つの関節点の現在・目標位置にもとづいて、関節の回転を計算
	if ( count == 3 )
	{
		// ３組みの２つのベクトルにもとづく回転を計算（各 src_vec の方向を dest_vec に向けるための３自由度の回転を計算）
		if ( !ComputeTripleVectorRotation( src_vec, dest_vec, rot_mat ) )
		{
			// 回転が求まらなかった場合は、
			// ｎ組みの２つのベクトルにもとづく回転を計算（src_vec の方向を dest_vec に向けるための３自由度の回転を計算）
			if ( !ComputeMultipleVectorRotation( 3, src_vec, dest_vec, rot_mat ) )
			{
				// 回転が求まらなかった場合は、
				// ２つのベクトルにもとづく回転を計算（sum_src_vec の方向を sum_dest_vec に向けるための２自由度の回転を計算）
				ComputeVectorRotation( sum_src_vec, sum_dest_vec, rot_mat );
			}
		}

		// 関節回転を設定
		pose.joint_rotations[ joint_no ] = rot_mat;
	}

	// 末端側の１つの関節点・体節点の現在・目標位置にもとづいて、関節の回転を計算
	//（複数の関節点や体節点が存在する場合は、平均の現在・目標位置にもとづいて、関節の回転を計算）
	else if ( count > 0 )
	{
		// ２つのベクトルにもとづく回転を計算（sum_src_vec の方向を sum_dest_vec に向けるための２自由度の回転を計算）
		ComputeVectorRotation( sum_src_vec, sum_dest_vec, rot_mat );

		// 関節回転を設定
		pose.joint_rotations[ joint_no ] = rot_mat;
	}

	// 末端側に一つも関節点・体節点が存在しない場合は、入力姿勢の関節の回転を保持する

	// 隣接する全ての末端側の関節に対して繰り返し
	for ( int i = 0; i < next_segment->num_joints; i ++ )
	{
		// 隣接する関節を取得（現在の関節はスキップ）
		next_joint = next_segment->joints[ i ];
		if ( next_joint->index == joint_no )
			continue;

		// 隣接する関節に対して再帰呼び出し
		ComputePostureFromJointPointsIteration( pose, next_joint->index, 
			joint_positions, joint_position_flags, segment_positions, segment_position_flags );
	}
}


//
//  ２つのベクトルにもとづく回転を計算（src_vec の方向を dest_vec に向けるための２自由度の回転を計算）
//
void  ComputeVectorRotation( const Vector3f & src_vec, const Vector3f & dest_vec, Matrix3f & rot )
{
	AxisAngle4f  rot_vec_angle;
	ComputeVectorRotation( src_vec, dest_vec, rot_vec_angle );
	rot.set( rot_vec_angle );
}


//
//  ２つのベクトルにもとづく回転を計算（src_vec の方向を dest_vec に向けるための２自由度の回転を計算）
//
void  ComputeVectorRotation( const Vector3f & src_vec, const Vector3f & dest_vec, AxisAngle4f & rot_vec_angle )
{
	Vector3f  axis;
	float  angle;


	// ※ レポート課題

	const float eps = 1e-6f;

	// 1) 回転軸 axis = src × dest
	axis.cross(src_vec, dest_vec);
	float axis_len = axis.length();
	if (axis_len < eps) {
		axis.set(1.0f, 0.0f, 0.0f);
	}
	else {
		axis.scale(1.0f / axis_len);
	}

	// 2) 回転角度 angle = acos((src·dest)/(|src||dest|))
	float src_len = src_vec.length();
	float dest_len = dest_vec.length();
	float cos_val;
	if (src_len * dest_len < eps) {
		cos_val = 1.0f;
	}
	else {
		cos_val = src_vec.dot(dest_vec) / (src_len * dest_len);
		// 手動クリッピング
		if (cos_val > 1.0f) cos_val = 1.0f;
		else if (cos_val < -1.0f) cos_val = -1.0f;
	}

    // 角度を計算
    angle = acosf(cos_val);
	// 回転軸・回転角度を設定
	rot_vec_angle.set( axis, angle );
}



//  ３組みの２つのベクトルにもとづく回転を計算（src_vecの方向をdest_vecに向けるための３自由度の回転を計算）
//
bool  ComputeTripleVectorRotation( const Vector3f src_vec[ 3 ], const Vector3f dest_vec[ 3 ], Matrix3f & rot )
{
	Matrix3f  rot0, rot1;

	// ３つのベクトルにもとづいて３×３行列を設定
	rot0.setColumn( 0, src_vec[ 0 ] );
	rot0.setColumn( 1, src_vec[ 1 ] );
	rot0.setColumn( 2, src_vec[ 2 ] );
	rot1.setColumn( 0, dest_vec[ 0 ] );
	rot1.setColumn( 1, dest_vec[ 1 ] );
	rot1.setColumn( 2, dest_vec[ 2 ] );


	// ※ レポート課題

    // rot0 の逆行列を計算することができなければ終了する（falseを返す）
    Matrix3f rot0_inv;
	if (fabs(rot0.determinant()) < 1.0e-6f) {
		return false; // rot0が特異な行列なので逆行列を求められない
	}
	rot0.invert(rot0_inv); // invert は void 関数として実行

    // rot0 の向き を rot1の向き に合わせるための回転行列を計算
    rot.mul(rot1, rot0_inv);

    // 回転行列の正規化
	rot.normalize();// normalize() が失敗したら

    // 回転行列の計算を終了する（trueを返す）
    return true;
}
    



//
//  ｎ組みの２つのベクトルにもとづく回転を計算（src_vec の方向を dest_vec に向けるための３自由度の回転を計算）
//
bool  ComputeMultipleVectorRotation( int num_vectors, const Vector3f * src_vec, const Vector3f * dest_vec, Matrix3f & rot )
{
	vector< Quat4f >  quats( num_vectors );
	vector< float >  weights( num_vectors );

	// ※ レポート課題

	// 各組の２つのベクトルにもとづく回転を計算
	for ( int i = 0; i < num_vectors; i++ )
	{
		Matrix3f tmp_rot;
		// ２つのベクトルにもとづく回転を計算（src_vec の方向を dest_vec に向けるための２自由度の回転を計算）
		ComputeVectorRotation( src_vec[i], dest_vec[i], tmp_rot );
		Quat4f q;
		q.set(tmp_rot);
		quats[i] = q;
	}

	// 各組の回転の重みを計算する（全ての組みを同じ重みで補間する）
	float w = 1.0f / static_cast<float>(num_vectors);
	for ( int i = 0; i < num_vectors; i++ )
	{
		weights[ i ] = w;
	}

	// 全ての組の回転を平均する
	Quat4f avg;
	MultipleRotationInterpolation(
		static_cast<int>(quats.size()),  // 要素数
		quats.data(),                    // Quat4f 配列へのポインタ
		weights.data(),                  // float 配列へのポインタ
		avg                              // 出力先
	);

	// 出力の回転を求める
	rot.set(avg);

	return  true;
}
