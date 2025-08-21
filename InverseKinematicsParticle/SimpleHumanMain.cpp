/**
***  キャラクタアニメーションのための人体モデルの表現・基本処理のサンプルプログラム
***  Copyright (c) 2015-, Masaki OSHITA (www.oshita-lab.org)
**/

/**
***  メイン関数
**/


// GLUTフレームワーク＋アプリケーション基底クラスの定義を読み込み
#include "SimpleHumanGLUT.h"

// アプリケーションの定義を読み込み
#include "InverseKinematicsParticleApp.h"



//
//  メイン関数（プログラムはここから開始）
//
int  main( int argc, char ** argv )
{
	// 全アプリケーションのリスト
	vector< class GLUTBaseApp * >    applications;

	// GLUTフレームワークのメイン関数を呼び出し（実行するアプリケーションを指定）
	SimpleHumanGLUTMain( new InverseKinematicsParticleApp(), argc, argv, "Inverse Kinematics (Particle)", 1280, 1024 );
}



