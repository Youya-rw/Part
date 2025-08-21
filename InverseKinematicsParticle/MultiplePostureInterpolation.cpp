/**
***  キャラクタアニメーションのための人体モデルの表現・基本処理のサンプルプログラム
***  Copyright (c) 2015-, Masaki OSHITA (www.oshita-lab.org)
**/

/**
***  複数姿勢補間
**/


// ヘッダファイルのインクルード
#include "SimpleHuman.h"
#include "MultiplePostureInterpolation.h"

// 標準算術関数・定数の定義
#define  _USE_MATH_DEFINES
#include <math.h>



//
//  複数姿勢補間処理（グローバル関数）
//


//
//  四元数を対数ベクトルに変換
//
void  LogQuatToVec( const Quat4f & q, Vector3f * v )
{
	float  angle, len;
	v->set( q.x, q.y, q.z );
	if ( q.w > 1.0f )
		angle = 0.0f;
	else if ( q.w < -1.0f )
		angle = (float) M_PI;
	else
		angle = acos( q.w );
	len = v->length();

	if ( len < 0.00000001f )
	{
		v->set( 0.0f, 0.0f, 0.0f );
		return;
	}

	v->scale( angle / len );
}


//
//  対数ベクトルを四元数に変換
//
void  ExpVecToQuat( const Vector3f & v, Quat4f * q )
{
	float  lv, sin_v, cos_v;
	lv = v.length();

	if ( lv < 0.00000001f )
	{
		q->set( 0.0f, 0.0f, 0.0f, 1.0f );
		return;
	}

	sin_v = sin( lv ) / lv;
	cos_v = cos( lv );
	q->set( v.x * sin_v, v.y * sin_v, v.z * sin_v, cos_v );
}


//
//  複数の四元数の固有平均を計算
//
Quat4f  AverageRotation( int num, const Quat4f * rotations )
{
	//	const float  eps = 0.0000001f;
	const float  eps = 0.001f;

	Quat4f  qr;

	// 初期値（QLERP補間）の計算
	qr.set( rotations[ 0 ] );
	for ( int i=0; i<num; i++ )
	{
		if ( rotations[ i ].w >= 0.0f )
			qr.add( rotations[ i ] );
		else
			qr.sub( rotations[ i ] );
	}
	if ( qr.w < eps )
		qr = rotations[ 0 ];
	else
		qr.normalize();

	// 繰り返し最適化計算
	Quat4f  duq, dr, ui, srcr;
	Vector3f  du, qv;
	do
	{
		du.set( 0.0f, 0.0f, 0.0f );
		ui.conjugate( qr );

		for ( int i=0; i<num; i++ )
		{
			srcr = rotations[ i ];
			if ( srcr.x * qr.x + srcr.y * qr.y + srcr.z * qr.z + srcr.w * qr.w < 0.0f )
				srcr.negate();

			dr.mul( ui, srcr );
			LogQuatToVec( dr, &qv );

			if ( isnan( qv.x ) )
				LogQuatToVec( dr, &qv );

			du.add( qv );
		}

		du.scale( 1.0f / num );
		ExpVecToQuat( du, &duq );

		qr.mul( qr, duq );
		qr.normalize();		
	}
	while( du.lengthSquared() > eps );

	return  qr;
}


//
//  複数回転補間（複数の回転（四元数）を補間）
//
void  MultipleRotationInterpolation( int num, const Quat4f * rotations, const float * weights, Quat4f & out_rotation )
{
	// 入力の全ての回転（四元数）の固有平均四元数を計算
	Quat4f  qa, qai;
	qa = AverageRotation( num, rotations );
	qai.inverse( qa );

	// 計算用変数
	Quat4f  q, qr;
	Vector3f  vec;
	
	// 対数ベクトルの加重平均
	Vector3f  avg_vec( 0.0f, 0.0f, 0.0f );

	// 複数の回転（四元数）の固有平均四元数からの差分の回転を対数ベクトルに変換して補間
	for ( int i = 0; i < num; i++ )
	{
		// ウェイトが０の回転はスキップ
		if ( weights[ i ] == 0.0f )
			continue;

		// 入力の i 番目の回転（四元数）を取得
		q = rotations[ i ];

		// ※ レポート課題

		// 固有平均四元数と入力の回転の方向が反対であれば、入力の回転を反転
//		if ( ??? )
//			q.negate();
 
		// 固有平均四元数に対する入力の回転の相対回転（四元数）を計算
//		qr = ???;

		// 相対回転（四元数）を対数ベクトルに変換
		LogQuatToVec( qr, &vec );

		// 入力のウェイトに応じて対数ベクトルを加重平均
//		avg_vec = ???;
	}

	// 対数ベクトルを回転（四元数）に変換
	ExpVecToQuat( avg_vec, &q );

	// 固有平均四元数に対して、求めた回転（四元数）を加えて、補間結果の回転を計算
//	out_rotation = ???;
	out_rotation = rotations[ 0 ];
}


//
//  複数姿勢補間（複数の姿勢を補間）
//
void  MultiplePostureInterpolation( int num_postures, const Posture ** postures, const float * weights, Posture & p )
{
	// 引数チェック（全ての姿勢の骨格モデルが同じでない場合は終了）
	for ( int i=0; i<num_postures; i++ )
		if ( postures[ i ]->body != p.body )
			return;

	// 入力姿勢が２つの場合や、入力姿勢は複数でも重みが設定されている姿勢が２つ以下のときには、
	// ２つの姿勢の補間処理を呼び出すようにすると、効率的に計算できる

	// 全姿勢の各関節の回転を補間
	for ( int i = 0; i < p.body->num_joints; i++ )
	{
		// ※ レポート課題
	}

	// 全姿勢のルートの向きを補間
	// ※ レポート課題

	// 全姿勢のルートの位置を補間
	// ※ レポート課題

}

