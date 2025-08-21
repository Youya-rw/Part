/**
***  キャラクタアニメーションのための人体モデルの表現・基本処理 ライブラリ・サンプルプログラム
***  Copyright (c) 2015-, Masaki OSHITA (www.oshita-lab.org)
***  Released under the MIT license http://opensource.org/licenses/mit-license.php
**/

/**
***  複数オブジェクトの位置・向きの操作機能
**/

#ifndef _OBJECT_LAYOUT_H_
#define _OBJECT_LAYOUT_H_


#include <vecmath.h>

#include <vector>
using namespace  std;


//
//  複数オブジェクトの位置・向きの操作機能 クラス
//
class  ObjectLayout
{
  public:
	/*  内部用構造体・列挙型の定義  */
	
	// 操作を表す列挙型
	enum  OperationEnum
	{
		OP_W_TRANSLATION,  // ワールド座標系での移動
		OP_M_TRANSLATION,  // モデル座標系での移動
		OP_W_ROTATION,     // ワールド座標系での回転
		OP_M_ROTATION,     // モデル座標系での回転
		OP_M_H_TRANS_ROT,  // モデル座標系での水平移動・回転
		OP_M_H_ROTAION,    // モデル座標系での水平回転
		NUM_OPERATION
	};

	// 操作のためのハンドル（各軸・平面）を表す列挙型
	enum  HandleEnum
	{
		H_NONE = -1,
		H_MODE_CHANGE,

		H_M_X_AXIS,
		H_M_Y_AXIS,
		H_M_Z_AXIS,
		H_M_XY_PLANE,
		H_M_YZ_PLANE,
		H_M_ZX_PLANE,

		H_W_X_AXIS,
		H_W_Y_AXIS,
		H_W_Z_AXIS,
		H_W_XY_PLANE,
		H_W_YZ_PLANE,
		H_W_ZX_PLANE,

		NUM_HANDLE
	};

	// 操作に対応するマウス座標軸を表す列挙型
	enum  MouseControlEnum
	{
		NO_MOUSE_CONTROL,
		MOUSE_PLUS_X,
		MOUSE_MINUS_X,
		MOUSE_PLUS_Y,
		MOUSE_MINUS_Y
	};

	// オブジェクト情報を表す構造体
	struct  Object
	{
		// 位置・向き
		Point3f   pos;
		Matrix3f  ori;

		// 物体を囲む直方体のサイズ（描画用）
		Vector3f  box_max;
		Vector3f  box_min;

		// 軸の長さ
		float  axis_length;

		// 適用可能な操作
		bool  allowed_op[ NUM_OPERATION ];

		// 以下は、計算用の変数

		Point2f  screen_pos;  // 画面上の座標
		Matrix4f  frame; // 位置・向き（キャッシュ用）
	};

	// 各種描画オプションを表す構造体
	struct  RenderOption
	{
		// オブジェクトの直方体の描画の有無と描画色
		bool  draw_box;
		Color3f  box_color;

		// 選択中のオブジェクトの直方体の描画の有無と描画色
		bool  draw_selected_box;
		Color3f  selected_box_color;

		// 移動・回転の操作軸の太さ（通常の軸と選択中の軸の太さ）
		float  axis_width;
		float  selected_axis_width;

		// 移動・回転の操作軸の長さ
		float  handle_length;

		// 移動・回転の操作軸の色
		Color3f  x_axis_color;
		Color3f  y_axis_color;
		Color3f  z_axis_color;

		// 座標系・操作軸の長さをオブジェクトの大きさに応じて変更
		bool  change_axis_length;

		// 座標系・操作軸の描画時に隠面消去を無視する
		bool  enable_xray_mode;

		// 描画する線の境界を滑らかにする
		bool  enable_smooth;
	};

  protected:
	/*  動作設定情報  */

	// 描画オプション
	RenderOption       render_option;

	// デフォルトのオブジェクト情報
	Object             default_object;

	// 選択判定を３次元空間で行うかどうかの設定
	bool               enable_3d_selection;

  protected:
	/*  オブジェクト情報  */
	
	// 全オブジェクトの情報
	vector< Object >  objects;

  protected:
	/*  操作情報  */
	
	// 現在選択中のオブジェクト番号
	int                curr_object;

	// 現在の操作モード
	OperationEnum      curr_operation;

	// ハンドル操作中かどうかのフラグ
	bool               on_control;
	bool               on_control_translation;
	bool               on_control_rotation;

	// 現在操作中のハンドル
	HandleEnum         active_handle;

	// 操作に対応するマウス座標軸
	MouseControlEnum   mouse_control;

	// マウス座標と移動平面の交点計算のための変数
	Vector3f           plane_norm;
	Point3f            plane_pos;

	// 操作ハンドルの画面上の座標
	Point2f            handle_pos[ NUM_HANDLE ];

	// 水平移動操作時のマウス座標のオフセット
	Point2f            trans_mouse_offest;

	// 前回のマウス座標
	int                last_mouse_x;
	int                last_mouse_y;


  public:
	/*  初期化・終了処理  */
	
	// コンストラクタ
	ObjectLayout();

  public:
	/*  アクセサ  */
	
	size_t  GetNumObjects() const { return  objects.size(); }
	bool  GetIsObjectSelected() const { return  ( curr_object != -1 ); }
	int  GetCurrentObject() const { return  curr_object; }
	bool  GetIsOnControl() const { return  on_control; }
	Point3f &  GetPosition( int no ) { return  objects[ no ].pos; }
	Matrix3f &  GetOrientation( int no ) { return  objects[ no ].ori; }
	Matrix4f &  GetFrame( int no );
	Matrix4f &  GetTransposedFrame( int no );
	const Point3f &  GetPosition( int no ) const { return  objects[ no ].pos; }
	const Matrix3f &  GetOrientation( int no ) const { return  objects[ no ].ori; }
	RenderOption &  GetRenderOption() { return  render_option; }
	Object &  GetDefaultObject() { return  default_object; }
	OperationEnum  GetOperationMode() { return  curr_operation; }
	const char *  GetOperationModeName();
	const float *  GetPositionArray( int no ) { return  & objects[ no ].pos.x; }
	const float *  GetOrientationArray( int no ) { return  & objects[ no ].ori.m00; }
	const float *  GetFrameArray( int no ) { GetFrame( no );  return  & objects[ no ].frame.m00; }
	const float *  GetTransposedFrameArray( int no ) { GetTransposedFrame( no );  return  & objects[ no ].frame.m00; }

  public:
	// オブジェクトの操作
	
	// オブジェクトの追加
	int  AddObject();
	int  AddObject( int no );

	// オブジェクトの削除
	int  DeleteObject();
	int  DeleteObject( int no );
	int  DeleteAllObjects();

	// オブジェクトの情報設定
	void  SetObjectPos( int no, const Point3f & p );
	void  SetObjectOri( int no, const Matrix3f & o );
	void  SetObjectFrame( int no, const Matrix4f & f );
	void  SetObjectSize( int no, const Vector3f & min, const Vector3f & max );
	void  SetObjectSize( int no, const Vector3f & s );
	void  SetObjectSize( int no, float s );
	void  SetObjectOperation( int no, bool flags[ NUM_OPERATION ] );
	void  SetObjectOperation( int no, OperationEnum op, bool flag );

	// オブジェクトの操作モードを変更
	bool  SetOperationMode( OperationEnum mode );

	// オブジェクトの操作モードを変更（次の操作モードにトグルで切替）
	void  ChanceNextOperationMode();

  public:
	// イベントハンドラ

	// オブジェクト位置・視点の変更を通知
	void  Update();

	// マウス移動時の処理
	void  OnMoveMouse( int mx, int my );

	// マウスのボタンが押された時の処理
	void  OnMouseDown( int mx, int my );

	// マウスのボタンが離された時の処理
	void  OnMouseUp( int mx, int my );

	// オブジェクトの操作情報の描画
	void  Render();

	// 全オブジェクトの直方体を描画
	void  RenderAllObjects();


  protected:
	// 内部メソッド

	// 全オブジェクトをスクリーン座標系に投影
	void  ProjectObjects();

	// マウス位置に近いオブジェクトを探索
	int  FindObject( int mx, int my, int threshold );
	int  FindObject2D( int mx, int my, int threshold );
	int  FindObject3D( int mx, int my );

	// マウス位置がオブジェクトと重なっているかを判定
	bool  SelectObject3D( int mx, int my, int object_id );

	// マウス位置に近いハンドルを探索
	HandleEnum  FindHandle( int mx, int my, OperationEnum op, int threshold );

	// マウス座標に対応する３次元空間の半直線を計算
	void  ComputeMouseRay( int mx, int my, Point3f & org, Vector3f & dir );

	// マウス座標に対応する３次元空間の半直線を計算
	static void  ComputeMouseRay( int mouse_x, int mouse_y, 
		const int gl_view[4], const float gl_model_mat[16], const float gl_project_mat[16], 
		Point3f & org, Vector3f & dir );

	// 半直線と三角面の交差判定
	static bool  RayTriCross( const Point3f & org, const Vector3f & dir,
		const Point3f & v1, const Point3f & v2, const Point3f & v3,
		Point3f & cross, float tri_ori = 0.0f );

	// 半直線と直方体の交差判定
	static bool  RayOBBCross( const Point3f & org, const Vector3f & dir,
		const Point3f & box_min, const Point3f & box_max, const Matrix4f & box_frame,
		Point3f & cross );

  public:
	// 描画の補助処理（クラス関数）

	// 線分の描画
	static void  DrawSegment( const Point3f & p0, const Point3f & p1, const Color3f & color );

	// 円弧の描画
	static void  DrawArc( const Point3f & v0, const Point3f & v1, const Color3f & color, int num_points );

	// 直方体の描画
	static void  DrawBox( const Matrix4f & frame, const Point3f & min_pos, const Point3f & max_pos, const Color3f & color );
};



#endif // _OBJECT_LAYOUT_H_
