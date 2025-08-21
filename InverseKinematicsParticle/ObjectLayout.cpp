/**
***  キャラクタアニメーションのための人体モデルの表現・基本処理 ライブラリ・サンプルプログラム
***  Copyright (c) 2015-, Masaki OSHITA (www.oshita-lab.org)
***  Released under the MIT license http://opensource.org/licenses/mit-license.php
**/

/**
***  複数オブジェクトの位置・向きの操作機能
**/


#include "ObjectLayout.h"

#include <GL/glut.h>

// 標準算術関数・定数の定義
#define  _USE_MATH_DEFINES
#include <math.h>


//
//  コンストラクタ
//
ObjectLayout::ObjectLayout()
{
	curr_object = -1;
	curr_operation = OP_W_TRANSLATION;
	active_handle = H_NONE;
	on_control = false;

	// 描画オプションの設定
	render_option.draw_box = true;
	render_option.box_color.set( 0.4f, 0.4f, 0.8f );
	render_option.draw_selected_box = true;
	render_option.selected_box_color.set( 0.8f, 0.0f, 0.8f );
	render_option.axis_width = 2.0f;
	render_option.selected_axis_width = 6.0f;
	render_option.handle_length = 0.7f;
	render_option.x_axis_color.set( 1.0f, 0.0f, 0.0f );
	render_option.y_axis_color.set( 0.0f, 1.0f, 0.0f );
	render_option.z_axis_color.set( 0.0f, 0.0f, 1.0f );
	render_option.change_axis_length = false;
	render_option.enable_xray_mode = true;
	render_option.enable_smooth = true;

	// デフォルトのオブジェクト情報を設定
	default_object.pos.set( 0.0f, 0.0f, 0.0f );
	default_object.ori.setIdentity();
	default_object.box_max.set( 0.5f, 0.5f, 0.5f );
	default_object.box_max.set( 0.5f, 0.5f, 0.5f );
	default_object.axis_length = 0.8f;
	default_object.allowed_op[ OP_W_TRANSLATION ] = true;
	default_object.allowed_op[ OP_M_TRANSLATION ] = true;
	default_object.allowed_op[ OP_W_ROTATION ] = true;
	default_object.allowed_op[ OP_M_ROTATION ] = true;
	default_object.allowed_op[ OP_M_H_TRANS_ROT ] = false;
	default_object.allowed_op[ OP_M_H_ROTAION ] = false;

	// 選択判定を３次元空間で行うかどうかの設定
	enable_3d_selection = false;
}


//
//  オブジェクトの位置・向きを表す変換行列を返す
//
Matrix4f &  ObjectLayout::GetFrame( int no )
{
	Object &  o = objects[ no ];
	o.frame.setIdentity();
	o.frame.set( o.ori, o.pos, 1.0f );
	return  o.frame;
}


//
//  オブジェクトの位置・向きを表す変換行列を返す
//
Matrix4f &  ObjectLayout::GetTransposedFrame( int no )
{
	Object &  o = objects[ no ];
	o.frame.setIdentity();
	o.frame.set( o.ori, o.pos, 1.0f );
	o.frame.transpose();
	return  o.frame;
}


//
//  現在の操作モードを表す文字列を返す
//
const char *  ObjectLayout::GetOperationModeName()
{
	// 各操作モードを表す文字列
	static const char *  operation_name[ NUM_OPERATION ] = {
		"Translation in World Coordinates", "Translation in Model Coordinates", 
		"Rotation in World Coordinates", "Rotation in Model Coordinates",
		"Horizontal Translation and Rotation", "Horizontal Rotation" };

	// オブジェクトが選択されていれば操作モードを表す文字列を返す
	if ( curr_object != -1 )
		return  operation_name[ curr_operation ];

	return  NULL;
}



//////////////////////////////////////////////////////////////////////////////
//
//  オブジェクトの操作
//


//
//  オブジェクトの追加
//
int  ObjectLayout::AddObject()
{
	objects.push_back( default_object );

	// スクリーン座標系に投影した座標を計算
	ProjectObjects();

	return  objects.size() - 1;
}


//
//  オブジェクトの削除
//
int  ObjectLayout::DeleteObject()
{
	objects.pop_back();
	if ( curr_object >= objects.size() )
		curr_object = -1;
	return  objects.size();
}

int  ObjectLayout::DeleteObject( int no )
{
	if ( objects.size() == 0 )
		return  0;

	for ( int i=no; i<objects.size()-1; i++ )
		objects[ i ] = objects[ i+1 ];
	objects.pop_back();

	curr_object = -1;

	return  objects.size();
}

int  ObjectLayout::DeleteAllObjects()
{
	objects.clear();
	curr_object = -1;
	return  0;
}


//
//  オブジェクトの情報設定
//
void  ObjectLayout::SetObjectPos( int no, const Point3f & p )
{
	objects[ no ].pos = p;
}

void  ObjectLayout::SetObjectOri( int no, const Matrix3f & o )
{
	objects[ no ].ori = o;
}

void  ObjectLayout::SetObjectFrame( int no, const Matrix4f & f )
{
	Vector3f  v;
	f.get( &v );
	objects[ no ].pos = v;

	f.get( &objects[ no ].ori );
}

void  ObjectLayout::SetObjectSize( int no, const Vector3f & min, const Vector3f & max )
{
	objects[ no ].box_max = max;
	objects[ no ].box_min = min;
	objects[ no ].axis_length = ( ( max.x > max.y ) ? ( ( max.x > max.z ) ? max.x : max.z ) : ( ( max.y > max.z) ? max.y : max.z ) ) * 0.8f;
}

void  ObjectLayout::SetObjectSize( int no, const Vector3f & s )
{
	objects[ no ].box_max.set( s.x * 0.5f, s.y * 0.5f, s.z * 0.5f );
	objects[ no ].box_min.set( s.x * -0.5f, s.y * -0.5f, s.z * -0.5f );
	objects[ no ].axis_length = ( ( s.x > s.y ) ? ( ( s.x > s.z ) ? s.x : s.z) : ( ( s.y > s.z ) ? s.y : s.z ) ) * 0.8f;
}

void  ObjectLayout::SetObjectSize( int no, float s )
{
	objects[ no ].box_max.set( s * 0.5f, s * 0.5f, s * 0.5f );
	objects[ no ].box_min.set( s * -0.5f, s * -0.5f, s * -0.5f );
	objects[ no ].axis_length = s * 0.8f; 
}

void  ObjectLayout::SetObjectOperation( int no, bool flags[ NUM_OPERATION ] )
{
	for ( int i=0; i<NUM_OPERATION; i++ )
		objects[ no ].allowed_op[ i ] = flags[ i ];
}

void  ObjectLayout::SetObjectOperation( int no, OperationEnum op, bool flag )
{
	objects[ no ].allowed_op[ op ] = flag;
}


//
//  オブジェクトの操作モードを変更
//
bool  ObjectLayout::SetOperationMode( OperationEnum mode )
{
	// 現在選択されているオブジェクトが、指定された操作モードに対応していなければ、変更しない
	if ( ( curr_object != -1 ) && !objects[ curr_object ].allowed_op[ mode ] )
		return  false;

	// 操作モードを変更
	curr_operation = mode;
	return  true;
}


//
//  オブジェクトの操作モードを変更（次の操作モードにトグルで切替）
//
void  ObjectLayout::ChanceNextOperationMode()
{
	for ( int i=0; i<NUM_OPERATION; i++ )
	{
		curr_operation = (OperationEnum)( ( curr_operation + 1 ) % NUM_OPERATION );

		// オブジェクトが選択されていないか、
		// 現在選択されているオブジェクトが、変更後の操作モードに対応していれば、終了
		// 対応していなければ、再度、処理を繰り返す
		if ( ( curr_object == -1 ) || objects[ curr_object ].allowed_op[ curr_operation ] )
			break;
	}
}



//////////////////////////////////////////////////////////////////////////////
//
//  イベントハンドラ
//


//
//  オブジェクト位置・視点の変更を通知
//
void  ObjectLayout::Update()
{
	// 全オブジェクトをスクリーン座標系に投影
	ProjectObjects();
}


//
//  マウスのボタンが押された時の処理
//
void  ObjectLayout::OnMouseDown( int mx, int my )
{
	// オブジェクトが選択されており、オブジェクトの上にマウスがあれば、操作モードを変更
	if ( ( curr_object != -1 ) && ( active_handle == H_MODE_CHANGE ) )
	{
		// 水平移動・回転モードの場合は、オブジェクトのドラッグで水平移動を行う（暫定）
		if ( curr_operation == OP_M_H_TRANS_ROT )
		{
			// 水平面での移動
			active_handle = H_W_ZX_PLANE;

			// 平面の情報設定
			plane_norm.set( 0.0f, 1.0f, 0.0f );
			plane_pos = objects[ curr_object ].pos;

			// 平行移動操作を有効に設定（２本の軸での移動）
			on_control = true;
			on_control_translation = true;
			on_control_rotation = false;

			// 現在のマウス座標を記録
			last_mouse_x = mx;
			last_mouse_y = my;
		}
		// 他の場合は、次の操作モードに変更
		else
			ChanceNextOperationMode();
	}

	// オブジェクトが選択されており、ハンドルの上にマウスがあれば、操作を開始
	else if ( ( curr_object != -1 ) && ( active_handle != H_NONE ) )
	{
		on_control = true;

		// 操作に対応するマウス座標軸を判定
		// ワールド座標軸orモデル座標軸での移動
		if ( ( ( active_handle >= H_M_X_AXIS ) && ( active_handle <= H_M_Z_AXIS ) ) ||
		     ( ( active_handle >= H_W_X_AXIS ) && ( active_handle <= H_W_Z_AXIS ) ) )
		{
			// 移動を行う軸がスクリーン上で横方向と縦方向のどちらに長いかにより判定
			Point2f  handle_delta;
			handle_delta = handle_pos[ active_handle ] - handle_pos[ 0 ];
			if ( fabs( handle_delta.x ) > fabs( handle_delta.y ) )
			{
				// 軸の方向により符号を判定
				if ( handle_delta.x > 0.0f )
					mouse_control = MOUSE_PLUS_X;
				else
					mouse_control = MOUSE_MINUS_X;
			}
			else
			{
				// 軸の方向により符号を判定
				if ( handle_delta.y > 0.0f )
					mouse_control = MOUSE_PLUS_Y;
				else
					mouse_control = MOUSE_MINUS_Y;
			}

			// 移動操作を有効に設定（１本の軸での移動）
			on_control_translation = true;
			on_control_rotation = false;
		}
		
		// ワールド座標軸orモデル座標軸での回転
		if ( ( ( active_handle >= H_M_XY_PLANE ) && ( active_handle <= H_M_ZX_PLANE ) ) ||
		     ( ( active_handle >= H_W_XY_PLANE ) && ( active_handle <= H_W_ZX_PLANE ) ) )
		{
			// 回転を行う方向を取得
			Point2f  handle_delta;
			switch ( active_handle )
			{
			  case  H_M_XY_PLANE:
				handle_delta.sub( handle_pos[ H_M_Y_AXIS ], handle_pos[ H_M_X_AXIS ] );  break;
			  case  H_M_YZ_PLANE:
				handle_delta.sub( handle_pos[ H_M_Z_AXIS ], handle_pos[ H_M_Y_AXIS ] );  break;
			  case  H_M_ZX_PLANE:
				handle_delta.sub( handle_pos[ H_M_X_AXIS ], handle_pos[ H_M_Z_AXIS ] );  break;
			  case  H_W_XY_PLANE:
				handle_delta.sub( handle_pos[ H_W_Y_AXIS ], handle_pos[ H_W_X_AXIS ] );  break;
			  case  H_W_YZ_PLANE:
				handle_delta.sub( handle_pos[ H_W_Z_AXIS ], handle_pos[ H_W_Y_AXIS ] );  break;
			  case  H_W_ZX_PLANE:
				handle_delta.sub( handle_pos[ H_W_X_AXIS ], handle_pos[ H_W_Z_AXIS ] );  break;
			}

			// 回転を行う方向がスクリーン上で横方向と縦方向のどちらに長いかにより判定
			if ( fabs( handle_delta.x ) > fabs( handle_delta.y ) )
			{
				// 回転を行う方向の向きにより符号を判定
				if ( handle_delta.x > 0.0f )
					mouse_control = MOUSE_PLUS_X;
				else
					mouse_control = MOUSE_MINUS_X;
			}
			else
			{
				// 回転を行う方向の向きにより符号を判定
				if ( handle_delta.y > 0.0f )
					mouse_control = MOUSE_PLUS_Y;
				else
					mouse_control = MOUSE_MINUS_Y;
			}

			// 回転操作を有効に設定（１本の軸での回転）
			on_control_translation = false;
			on_control_rotation = true;
		}

		// 現在のマウス座標を記録
		last_mouse_x = mx;
		last_mouse_y = my;
	}

	// オブジェクトを選択
	else
	{
		// クリック位置に近いオブジェクトを探索
		int  find_object = -1;
		find_object = FindObject( mx, my, 10 );
		curr_object = find_object;

		// オブジェクトの操作モードを変更
		//（選択されたオブジェクトが現在の操作モードに対応していなければ、次のモードに切替）
		if ( !SetOperationMode( curr_operation ) )
			ChanceNextOperationMode();

		// ハンドラのスクリーン座標を計算
		ProjectObjects();
	}
}


//
//  マウスのボタンが離された時の処理
//
void  ObjectLayout::OnMouseUp( int mx, int my )
{
	// 現在の操作を解除する
	on_control = false;
	on_control_translation = false;
	on_control_rotation = false;
	trans_mouse_offest.set( 0.0f, 0.0f );
}


//
//  マウス移動時の処理
//
void  ObjectLayout::OnMoveMouse( int mx, int my )
{
	const float  pos_scale = 0.01f;
	const float  angle_scale = 0.01f;

	// オブジェクトの位置・向きをコントロール
	if ( on_control )
	{
		Object &  o = objects[ curr_object ];
		int  dx, dy;
		dx = mx - last_mouse_x;
		dy = my - last_mouse_y;

		// マウス座標に対応する移動平面の座標から位置を計算
		if ( on_control_translation && ( active_handle == H_W_ZX_PLANE ) ) 
		{
			// マウス座標に対応する３次元空間の半直線を計算
			Vector3f  eye_dir;
			Point3f   eye_pos;
			ComputeMouseRay( mx - trans_mouse_offest.x, my - trans_mouse_offest.y, eye_pos, eye_dir );
			
			// 現在の部位の位置を含み視線に垂直な面と、マウス座標に対応する半直線の交点を計算
			Point3f  pos;
			float  t = ( plane_norm.dot( plane_pos ) - plane_norm.dot( eye_pos ) ) / plane_norm.dot( eye_dir );
			pos.scaleAdd( t, eye_dir, eye_pos );

			o.pos = pos;
		}

		// マウス座標変化に応じて移動量を計算
		else if ( on_control_translation ) 
		{
			// 移動を行う軸を取得
			Vector3f  vec;
			switch ( active_handle )
			{
			  case  H_W_X_AXIS:
				vec.set( 1.0f, 0.0f, 0.0f );  break;
			  case  H_W_Y_AXIS:
				vec.set( 0.0f, 1.0f, 0.0f );  break;
			  case  H_W_Z_AXIS:
				vec.set( 0.0f, 0.0f, 1.0f );  break;
			  case  H_M_X_AXIS:
				o.ori.getColumn( 0, &vec );  break;
			  case  H_M_Y_AXIS:
				o.ori.getColumn( 1, &vec );  break;
			  case  H_M_Z_AXIS:
				o.ori.getColumn( 2, &vec );  break;
			}

			// 操作に対応するマウス座標軸に応じて移動を適用
			if ( mouse_control == MOUSE_PLUS_X )
				o.pos.scaleAdd( dx * pos_scale, vec, o.pos );
			else if ( mouse_control == MOUSE_MINUS_X )
				o.pos.scaleAdd( -dx * pos_scale, vec, o.pos );
			else if ( mouse_control == MOUSE_PLUS_Y )
				o.pos.scaleAdd( dy * pos_scale, vec, o.pos );
			else if ( mouse_control == MOUSE_MINUS_Y )
				o.pos.scaleAdd( -dy * pos_scale, vec, o.pos );
		}

		// マウス座標変化に応じて回転量を計算
		if ( on_control_rotation ) 
		{
			// 操作に対応するマウス座標軸に応じて回転角度を計算
			float  angle = 0.0f;
			if ( mouse_control == MOUSE_PLUS_X )
				angle = dx * angle_scale;
			else if ( mouse_control == MOUSE_PLUS_Y )
				angle = dy * angle_scale;
			else if ( mouse_control == MOUSE_MINUS_X )
				angle = -dx * angle_scale;
			else if ( mouse_control == MOUSE_MINUS_Y )
				angle = -dy * angle_scale;

			// 回転行列を適用
			Matrix3f  rot;
			switch ( active_handle )
			{
			  case  H_M_XY_PLANE:
				rot.rotZ( angle );  o.ori.mul( o.ori, rot );  break;
			  case  H_M_YZ_PLANE:
				rot.rotX( angle );  o.ori.mul( o.ori, rot );  break;
			  case  H_M_ZX_PLANE:
				rot.rotY( angle );  o.ori.mul( o.ori, rot );  break;
			  case  H_W_XY_PLANE:
				rot.rotZ( angle );  o.ori.mul( rot, o.ori );  break;
			  case  H_W_YZ_PLANE:
				rot.rotX( angle );  o.ori.mul( rot, o.ori );  break;
			  case  H_W_ZX_PLANE:
				rot.rotY( angle );  o.ori.mul( rot, o.ori );  break;
			}
		}

		// 変換行列を更新
		o.frame.set( o.ori, o.pos, 1.0f );

		// オブジェクト・ハンドルのスクリーン上の座標を更新
		ProjectObjects();

		// 現在のマウス座標を記録
		last_mouse_x = mx;
		last_mouse_y = my;
	}

	// マウス座標に近いハンドルを選択
	else if ( curr_object != -1 )
	{
		// マウス座標に近いハンドルを検索
		HandleEnum  handle = FindHandle( mx, my, curr_operation, 24 );

		// 移動操作モードはハンドルを
		active_handle = handle;

		// 水平移動操作時のマウス座標のオフセットを計算
		if ( curr_operation == OP_M_H_TRANS_ROT )
		{
			trans_mouse_offest.set( mx, my );
			trans_mouse_offest.sub( handle_pos[ H_MODE_CHANGE ] );
		}
	}
}


//
//  オブジェクトの操作情報の描画
//
void  ObjectLayout::Render()
{
	// オブジェクトが選択されていなければ終了
	if ( curr_object == -1 )
		return;

	// 現在のレンダリング設定を記録
	GLboolean  b_lighting, b_depth_test, b_blend, b_line_smooth;
	glGetBooleanv( GL_LIGHTING, &b_lighting );
	glGetBooleanv( GL_DEPTH_TEST, &b_depth_test );
	glGetBooleanv( GL_BLEND, &b_blend );
	glGetBooleanv( GL_LINE_SMOOTH, &b_line_smooth );
	float  line_width;
	glGetFloatv( GL_LINE_WIDTH, &line_width );		

	// 描画オプションに応じてレンダリング設定
	const RenderOption &  ro = render_option;
	glDisable( GL_LIGHTING );
	if ( ro.enable_smooth )
	{
		glEnable( GL_BLEND );
		glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
		glEnable( GL_LINE_SMOOTH );
	}

	// 選択（描画）オブジェクトの情報取得
	Object &  o = objects[ curr_object ];
	Matrix4f  frame;
	frame.set( o.ori, o.pos, 1.0f );

	// 各軸の長さ（全軸で共通の長さを使用 or 物体のサイズを使用）
	float  sx, sy, sz;
	if ( ro.change_axis_length )
	{
		sx = ( o.box_max.x >= -o.box_min.x ) ? o.box_max.x : -o.box_min.x;
		sy = ( o.box_max.y >= -o.box_min.y ) ? o.box_max.y : -o.box_min.y;
		sz = ( o.box_max.z >= -o.box_min.z ) ? o.box_max.z : -o.box_min.z;
	}
	else
	{
		sx = o.axis_length;
		sy = o.axis_length;
		sz = o.axis_length;
	}

	// 原点と各軸の末端位置を設定
	Point3f  handle_o, handle_x, handle_y, handle_z;
	handle_o.set( 0.0f, 0.0f, 0.0f );
	handle_x.set( sx, 0.0f, 0.0f );
	handle_y.set( 0.0f, sy, 0.0f );
	handle_z.set( 0.0f, 0.0f, sz );

	// バウンディングボックスはＺバッファを考慮して描画
	glEnable( GL_DEPTH_TEST );

	// 選択中のオブジェクトのバウンディングボックスを描画
	if ( ro.draw_selected_box )
		DrawBox( frame, o.box_min, o.box_max, ro.selected_box_color );

	// 座標系の描画でＺバッファを考慮するかどうかを設定
	if ( ro.enable_xray_mode )
		glDisable( GL_DEPTH_TEST );

	// 位置操作のための座標系・操作軸を描画
	if ( ( curr_operation == OP_W_TRANSLATION ) || ( curr_operation == OP_M_TRANSLATION ) ||
		    ( curr_operation == OP_M_H_TRANS_ROT ) )
	{
		glPushMatrix();

		// モデル座標系とワールド座標系のどちらで描画するかを設定
		if ( curr_operation == OP_W_TRANSLATION )
		{
			// ワールド座標系で描画
			glTranslatef( o.pos.x, o.pos.y, o.pos.z );
		}
		if ( ( curr_operation == OP_M_TRANSLATION ) || ( curr_operation == OP_M_H_TRANS_ROT ) )
		{
			// モデル座標系で描画
			Matrix4f  mat;
			mat.transpose( frame );
			glMultMatrixf( &mat.m00 );
		}

		// 移動の操作軸を描画（X軸方向の移動）
		glLineWidth( ( active_handle == H_W_X_AXIS ) || ( active_handle == H_M_X_AXIS ) ? ro.selected_axis_width : ro.axis_width );
		DrawSegment( handle_o, handle_x, ro.x_axis_color );

		// 移動を行う方向を表す線分を描画（Y軸方向の移動）
		if ( curr_operation != OP_M_H_TRANS_ROT )
		{
			glLineWidth( ( active_handle == H_W_Y_AXIS ) || ( active_handle == H_M_Y_AXIS ) ? ro.selected_axis_width : ro.axis_width );
			DrawSegment( handle_o, handle_y, ro.y_axis_color );
		}

		// 移動を行う方向を表す線分を描画（Z軸方向の移動）
		glLineWidth( ( active_handle == H_W_Z_AXIS ) || ( active_handle == H_M_Z_AXIS ) ? ro.selected_axis_width : ro.axis_width );
		DrawSegment( handle_o, handle_z, ro.z_axis_color );

		glPopMatrix();
	}

	// 回転操作のための座標系・操作軸を描画
	if ( ( curr_operation == OP_W_ROTATION ) || ( curr_operation == OP_M_ROTATION ) || 
		    ( curr_operation == OP_M_H_TRANS_ROT ) || ( curr_operation == OP_M_H_ROTAION ) )
	{
		glPushMatrix();

		// モデル座標系とワールド座標系のどちらで描画するかを設定
		if ( curr_operation == OP_W_ROTATION )
		{
			// ワールド座標系で描画
			glTranslatef( o.pos.x, o.pos.y, o.pos.z );
		}
		if ( ( curr_operation == OP_M_ROTATION ) || ( curr_operation == OP_M_H_TRANS_ROT ) || ( curr_operation == OP_M_H_ROTAION ) )
		{
			// モデル座標系で描画
			Matrix4f  mat;
			mat.transpose( frame );
			glMultMatrixf( &mat.m00 );
		}

		// 座標系を描画
		if ( ( curr_operation != OP_M_H_TRANS_ROT ) && ( curr_operation != OP_M_H_ROTAION ) )
		{
			Color3f  black( 0.0f, 0.0f, 0.0f );
			glLineWidth( ro.axis_width );
			DrawSegment( handle_o, handle_x, black );
			DrawSegment( handle_o, handle_y, black );
			DrawSegment( handle_o, handle_z, black );
		}

		// 円弧の描画のための設定・変数
		const int  num_arc_points = 6;
		Vector3f  arc_vec0, arc_vec1;
		float  s0, c0, s1, c1;
		s0 = sin( 20 * M_PI / 180.0 );
		c0 = cos( 20 * M_PI / 180.0 );
		s1 = sin( 70 * M_PI / 180.0 );
		c1 = cos( 70 * M_PI / 180.0 );

		// 回転を行う方向を表す円弧を描画（X軸周りの回転）
		if ( ( curr_operation != OP_M_H_TRANS_ROT ) && ( curr_operation != OP_M_H_ROTAION ) )
		{
			arc_vec0.set( 0.0f, sy * c0, sz * s0 );
			arc_vec1.set( 0.0f, sy * c1, sz * s1 );
			glLineWidth( ( active_handle == H_M_YZ_PLANE ) || ( active_handle == H_W_YZ_PLANE ) ? ro.selected_axis_width : ro.axis_width );
			DrawArc( arc_vec0, arc_vec1, ro.x_axis_color, num_arc_points );
		}

		// 回転を行う方向を表す円弧を描画（Y軸周りの回転）
		arc_vec0.set( sx * c0, 0.0f, sz * s0 );
		arc_vec1.set( sx * c1, 0.0f, sz * s1 );
		glLineWidth( ( active_handle == H_M_ZX_PLANE ) || ( active_handle == H_W_ZX_PLANE ) ? ro.selected_axis_width : ro.axis_width );
		DrawArc( arc_vec0, arc_vec1, ro.y_axis_color, num_arc_points );

		// 回転を行う方向を表す円弧を描画（Z軸周りの回転）
		if ( ( curr_operation != OP_M_H_TRANS_ROT ) && ( curr_operation != OP_M_H_ROTAION ) )
		{
			arc_vec0.set( sx * c0, sy * s0, 0.0f );
			arc_vec1.set( sx * c1, sy * s1, 0.0f );
			glLineWidth( ( active_handle == H_M_XY_PLANE ) || ( active_handle == H_W_XY_PLANE ) ? ro.selected_axis_width : ro.axis_width );
			DrawArc( arc_vec0, arc_vec1, ro.z_axis_color, num_arc_points );
		}

		glPopMatrix();
	}

	// レンダリング設定を復元
	if ( b_lighting )
		glEnable( GL_LIGHTING );
	if ( b_depth_test )
		glEnable( GL_DEPTH_TEST );
	else
		glDisable( GL_DEPTH_TEST );
	if ( !b_blend )
		glDisable( GL_BLEND );
	if ( !b_line_smooth )
		glDisable( GL_LINE_SMOOTH );
	glLineWidth( line_width );
}


//
//  全オブジェクトの直方体を描画
//
void  ObjectLayout::RenderAllObjects()
{
	// 現在のレンダリング設定を記録
	GLboolean  b_lighting, b_depth_test, b_blend, b_line_smooth;
	glGetBooleanv( GL_LIGHTING, &b_lighting );
	glGetBooleanv( GL_DEPTH_TEST, &b_depth_test );
	glGetBooleanv( GL_BLEND, &b_blend );
	glGetBooleanv( GL_LINE_SMOOTH, &b_line_smooth );

	// 描画オプションに応じてレンダリング設定
	const RenderOption &  ro = render_option;
	glDisable( GL_LIGHTING );
	if ( ro.enable_smooth )
	{
		glEnable( GL_BLEND );
		glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
		glEnable( GL_LINE_SMOOTH );
	}

	// 全オブジェクトの直方体を描画
	for ( int i=0; i<objects.size(); i++)
	{
		Object &  o = objects[ i ];
		Matrix4f  frame;
		frame.set( o.ori, o.pos, 1.0f );

		// 各軸の長さ（全軸で共通の長さを使用 or 物体のサイズを使用）
		float  sx, sy, sz;
		if ( ro.change_axis_length )
		{
			sx = ( o.box_max.x >= -o.box_min.x ) ? o.box_max.x : -o.box_min.x;
			sy = ( o.box_max.y >= -o.box_min.y ) ? o.box_max.y : -o.box_min.y;
			sz = ( o.box_max.z >= -o.box_min.z ) ? o.box_max.z : -o.box_min.z;
		}
		else
		{
			sx = o.axis_length;  sy = o.axis_length;  sz = o.axis_length;
		}

		if ( !ro.draw_box )
			continue;

		// バウンディングボックスはＺバッファを考慮して描画
		glEnable( GL_DEPTH_TEST );

		// オブジェクトのバウンディングボックスを描画
		DrawBox( frame, o.box_min, o.box_max, ro.box_color );
	}

	// レンダリング設定を復元
	if ( b_lighting )
		glEnable( GL_LIGHTING );
	if ( b_depth_test )
		glEnable( GL_DEPTH_TEST );
	else
		glDisable( GL_DEPTH_TEST );
	if ( !b_blend )
		glDisable( GL_BLEND );
	if ( !b_line_smooth )
		glDisable( GL_LINE_SMOOTH );
}



//////////////////////////////////////////////////////////////////////////////
//
//  内部メソッド
//


//
//  全オブジェクトをスクリーン座標系に投影
//
void  ObjectLayout::ProjectObjects()
{
	// OpenGL の変換行列を取得
	double  model_view_matrix[ 16 ];
	double  projection_matrix[ 16 ];
	int  viewport_param[ 4 ];
	Point3d  projected_pos;
	glGetDoublev( GL_MODELVIEW_MATRIX, model_view_matrix );
	glGetDoublev( GL_PROJECTION_MATRIX, projection_matrix );
	glGetIntegerv( GL_VIEWPORT, viewport_param );

	// ワールド座標をスクリーン座標に投影するためのマクロ
	#define  PROJECT( wp, sp )	gluProject( wp.x, wp.y, wp.z, \
			model_view_matrix, projection_matrix, viewport_param, \
			&projected_pos.x, &projected_pos.y, &projected_pos.z ); \
		sp.x = projected_pos.x; \
		sp.y = viewport_param[ 3 ] - projected_pos.y;

	// 各オブジェクトの位置をスクリーン座標に投影
	for ( int i=0; i<objects.size(); i++ )
	{
		Object &  o = objects[ i ];
		PROJECT( o.pos, o.screen_pos );
	}

	// 各操作ハンドルの位置をスクリーン座標に投影
	if ( curr_object != -1 )
	{
		Object &  o = objects[ curr_object ];
		Vector3f  x_axis, y_axis, z_axis, pos;
		float  len = render_option.handle_length;

		// 各軸の長さ（全軸で共通の長さを使用 or 物体のサイズを使用）
		float  sx, sy, sz;
		if ( render_option.change_axis_length )
		{
			sx = ( o.box_max.x >= -o.box_min.x ) ? o.box_max.x : -o.box_min.x;
			sy = ( o.box_max.y >= -o.box_min.y ) ? o.box_max.y : -o.box_min.y;
			sz = ( o.box_max.z >= -o.box_min.z ) ? o.box_max.z : -o.box_min.z;
		}
		else
		{
			sx = o.axis_length;  sy = o.axis_length;  sz = o.axis_length;
		}

		handle_pos[ H_MODE_CHANGE ] = o.screen_pos;

		o.ori.getColumn( 0, &x_axis );
		o.ori.getColumn( 1, &y_axis );
		o.ori.getColumn( 2, &z_axis );
		pos.scaleAdd( sx, x_axis, o.pos );
		PROJECT( pos, handle_pos[ H_M_X_AXIS ] );
		pos.scaleAdd( sy, y_axis, o.pos );
		PROJECT( pos, handle_pos[ H_M_Y_AXIS ] );
		pos.scaleAdd( sz, z_axis, o.pos );
		PROJECT( pos, handle_pos[ H_M_Z_AXIS ] );
		pos.scaleAdd( sx * len, x_axis, o.pos );
		pos.scaleAdd( sy * len, y_axis, pos );
		PROJECT( pos, handle_pos[ H_M_XY_PLANE ] );
		pos.scaleAdd( sy * len, y_axis, o.pos );
		pos.scaleAdd( sz * len, z_axis, pos );
		PROJECT( pos, handle_pos[ H_M_YZ_PLANE ] );
		pos.scaleAdd( sz * len, z_axis, o.pos );
		pos.scaleAdd( sx * len, x_axis, pos );
		PROJECT( pos, handle_pos[ H_M_ZX_PLANE ] );

		x_axis.set( 1.0f, 0.0f, 0.0f );
		y_axis.set( 0.0f, 1.0f, 0.0f );
		z_axis.set( 0.0f, 0.0f, 1.0f );
		pos.scaleAdd( sx, x_axis, o.pos );
		PROJECT( pos, handle_pos[ H_W_X_AXIS ] );
		pos.scaleAdd( sy, y_axis, o.pos );
		PROJECT( pos, handle_pos[ H_W_Y_AXIS ] );
		pos.scaleAdd( sz, z_axis, o.pos );
		PROJECT( pos, handle_pos[ H_W_Z_AXIS ] );
		pos.scaleAdd( sx * len, x_axis, o.pos );
		pos.scaleAdd( sy * len, y_axis, pos );
		PROJECT( pos, handle_pos[ H_W_XY_PLANE ] );
		pos.scaleAdd( sy * len, y_axis, o.pos );
		pos.scaleAdd( sz * len, z_axis, pos );
		PROJECT( pos, handle_pos[ H_W_YZ_PLANE ] );
		pos.scaleAdd( sz * len, z_axis, o.pos );
		pos.scaleAdd( sx * len, x_axis, pos );
		PROJECT( pos, handle_pos[ H_W_ZX_PLANE ] );
	}
}


//
//  マウス位置に近いオブジェクトを探索
//
int  ObjectLayout::FindObject( int mx, int my, int threshold )
{
	if ( enable_3d_selection )
		return  FindObject3D( mx, my );
	else
		return  FindObject2D( mx, my, threshold );
}


//
//  マウス位置に近いオブジェクトを探索
//
int  ObjectLayout::FindObject2D( int mx, int my, int threshold )
{
	int  closest_no = -1;
	int  closest_dist;
	int  dist, dx, dy;

	// 各オブジェクトのマウス位置の距離を計算して、最も近いものを探索
	for ( int i=0; i<objects.size(); i++ )
	{
		// 距離を計算
		const Object &  o = objects[ i ];
		dx = o.screen_pos.x - mx;
		dy = o.screen_pos.y - my;
		dist = dx * dx + dy * dy;

		// 閾値以下でなければスキップ
		if ( dist > threshold * threshold )
			continue;

		// これまでに見つかったオブジェクトより近ければ更新
		if ( ( closest_no == -1 ) || ( dist < closest_dist ) )
		{
			closest_no = i;
			closest_dist = dist;
		}
	}

	// 見つかったオブジェクト番号を返す
	return  closest_no;
}


//
//  マウス位置に近いオブジェクトを探索
//
int  ObjectLayout::FindObject3D( int mx, int my )
{
	int  closest_no = -1;
	int  closest_dist;
	int  dist;

	// マウス座標に対応する３次元空間の半直線を計算
	Point3f   eye_pos;
	Vector3f  eye_dir;
	ComputeMouseRay( mx, my, eye_pos, eye_dir );

	// 各オブジェクトのマウス位置の距離を計算して、最も近いものを探索
	bool  cross;
	Point3f  cross_pos;
	Vector3f  vec;
	for ( int i=0; i<objects.size(); i++ )
	{
		// 半直線と直方体の交差判定
		const Object &  o = objects[ i ];
		cross = RayOBBCross( eye_pos, eye_dir, o.box_min, o.box_max, o.frame, cross_pos );
		if ( !cross )
			continue;

		// 距離を計算
		vec.sub( cross_pos, eye_pos );
		dist = vec.length();

		// これまでに見つかったオブジェクトより近ければ更新
		if ( ( closest_no == -1 ) || ( dist < closest_dist ) )
		{
			closest_no = i;
			closest_dist = dist;
		}
	}

	// 見つかったオブジェクト番号を返す
	return  closest_no;
}


//
//  マウス位置がオブジェクトと重なっているかを判定
//
bool  ObjectLayout::SelectObject3D( int mx, int my, int object_id )
{
	if ( ( object_id < 0 ) || ( object_id > objects.size() ) )
		return  false;

	// マウス座標に対応する３次元空間の半直線を計算
	Point3f   eye_pos;
	Vector3f  eye_dir;
	ComputeMouseRay( mx, my, eye_pos, eye_dir );

	// 半直線と直方体の交差判定
	const Object &  o = objects[ object_id ];
	bool  cross;
	Point3f  cross_pos;
	cross = RayOBBCross( eye_pos, eye_dir, o.box_min, o.box_max, o.frame, cross_pos );

	return  cross;
}



//
//  マウス位置に近いハンドルを探索
//
ObjectLayout::HandleEnum  ObjectLayout::FindHandle( int mx, int my, OperationEnum op, int threshold )
{
	int  closest_no = H_NONE;
	int  closest_dist;
	int  dist, dx, dy;

	// 各ハンドルとマウス位置の間の距離を計算して、最も近いハンドルを探索
	for ( int i=0; i<NUM_HANDLE; i++ )
	{
		// 現在の操作が対象とするハンドルでなければスキップ
		if ( ( ( op == OP_W_TRANSLATION ) || ( op == OP_W_ROTATION ) ) &&
		     ( i >= H_M_X_AXIS ) && ( i <= H_M_ZX_PLANE ) )
			continue;
		if ( ( ( op == OP_M_TRANSLATION ) || ( op == OP_M_ROTATION ) ) &&
		     ( i >= H_W_X_AXIS ) && ( i <= H_W_ZX_PLANE ) )
			continue;
		if ( ( op == OP_M_H_TRANS_ROT ) &&
		     ( i != H_M_X_AXIS ) && ( i != H_M_Z_AXIS ) && ( i != H_M_ZX_PLANE ) && ( i != H_MODE_CHANGE ) )
			continue;
		if ( ( op == OP_M_H_ROTAION ) &&
		     ( i != H_M_ZX_PLANE ) && ( i != H_MODE_CHANGE ) )
			continue;

		// 距離を計算（暫定）
		dx = handle_pos[ i ].x - mx;
		dy = handle_pos[ i ].y - my;
		dist = dx * dx + dy * dy;

		// 閾値以下でなければスキップ
		if ( dist > threshold * threshold )
			continue;

		// これまでに見つかったオブジェクトより近ければ更新
		if ( ( closest_no == -1 ) || ( dist < closest_dist ) )
		{
			closest_no = i;
			closest_dist = dist;
		}
	}

	// マウス位置がオブジェクト上にある場合かどうかを判定
	if ( enable_3d_selection )
	{
//		if ( FindObject3D( mx, my ) == curr_object )
		if ( curr_object != -1 )
			if ( SelectObject3D( mx, my, curr_object ) )
				closest_no = H_MODE_CHANGE;
	}

	// 見つかったハンドル番号を返す
	return  (HandleEnum) closest_no;
}



//
//  マウス座標に対応する３次元空間の半直線を計算
//
void  ObjectLayout::ComputeMouseRay( 
	int mx, int my, Point3f & org, Vector3f & dir )
{
	// ピューポート、ワールド座標系行列、射影行列を取得
	int  view[ 4 ];
	float  model[ 16 ], project[ 16 ];
	glGetIntegerv( GL_VIEWPORT, view );
	glGetFloatv( GL_MODELVIEW_MATRIX, model );
	glGetFloatv( GL_PROJECTION_MATRIX, project );

	// ビューポートに応じて座標値を修正（暫定）
	my += view[ 1 ];

	// マウス座標に対応する３次元空間の半直線を計算
	ComputeMouseRay( mx, my, view, model, project, org, dir );
}


//
//  マウス座標に対応する３次元空間の半直線を計算
//
void  ObjectLayout::ComputeMouseRay( 
	int mouse_x, int mouse_y, 
	const int view[4], const float model[16], const float project[16], 
	Point3f & org, Vector3f & dir )
{
	// 頂点座標を計算
	float  fx, fy;
	float  view_x = view[ 0 ], view_y = view[ 1 ], view_w = view[ 2 ], view_h = view[ 3 ];
	fx = 2.0 * ( mouse_x - view_x ) / view_w - 1.0;
	fy = 2.0 * ( view_h - mouse_y ) / view_h - 1.0;
	fx = fx / project[ 4*0 + 0 ];
	fy = fy / project[ 4*1 + 1 ];

	// ワールド座標系行列の逆行列を計算
	Matrix4f  mat;
	mat.set( model );
	mat.transpose();
	mat.invert();

	// スクリーン座標をワールド座標の半直線に変換
	org.set( 0.0f, 0.0f, 0.0f );
	dir.set( fx, fy, -1.0f );
	mat.transform( &org );
	mat.transform( &dir );
}


//
//  半直線と三角面の交差判定
//
bool  ObjectLayout::RayTriCross( 
	const Point3f & org, const Vector3f & dir,
	const Point3f & v1, const Point3f & v2, const Point3f & v3,
	Point3f & cross,
	float tri_ori // 正のときは半時計周り、負のときは時計周り、0のときは両方の面と判定（省略可）
)
{
	// 三角面を含む超平面の法線計算
	Vector3f  norm;
	norm.cross( v2 - v1, v3 - v1 );

	// 面の向きを判定
	if ( tri_ori != 0.0f )
	{
		// 視線の向きと法線の向きにより、法線が視点側を向いているか判定
		if ( norm.dot( dir ) * tri_ori > 0.0f )
			return  false;
	}

	// 半直線を含む直線と三角面を含む超平面の交点を計算
	float  t, nd, np;
	nd = norm.dot( dir );
	np = norm.dot( v1 - org );
	t = np / nd;

	// 直線と超平面がほぼ平行の場合は交差しないと判定
	if ( fabs( nd ) < 0.0001f )
		return  false;

	// 交点が半直線の上になければ交差しない
	if ( t < 0.0f )
		return  false;

	// 交点が三角面の内側にあるかどうかを判定
	Point3f  p;
	Vector3f  seg, seg_p, seg_inside;
	p.scaleAdd( t, dir, org );
	seg.sub( v2, v1 );
	seg_p.sub( p, v1 );
	seg_inside.cross( norm, seg );
	if ( seg_inside.dot( seg_p ) < 0.0f )
		return  false;
	seg.sub( v3, v2 );
	seg_p.sub( p, v2 );
	seg_inside.cross( norm, seg );
	if ( seg_inside.dot( seg_p ) < 0.0f )
		return  false;
	seg.sub( v1, v3 );
	seg_p.sub( p, v3 );
	seg_inside.cross( norm, seg );
	if ( seg_inside.dot( seg_p ) < 0.0f )
		return  false;

	// 交点を返す
	cross.set( p );
	return  true;
}


//
//  半直線と直方体の交差判定
//
bool  ObjectLayout::RayOBBCross( const Point3f & org, const Vector3f & dir,
	const Point3f & box_min, const Point3f & box_max, const Matrix4f & box_frame,
	Point3f & cross )
{
	// 座標変換後の直方体の座標を計算
	Point3f   box_org;
	Vector3f  box_axis_x, box_axis_y, box_axis_z;
	box_org = box_min;
	box_axis_x.set( box_max.x - box_min.x, 0.0f, 0.0f );
	box_axis_y.set( 0.0f, box_max.y - box_min.y, 0.0f );
	box_axis_z.set( 0.0f, 0.0f, box_max.z - box_min.z );
	box_frame.transform( &box_org );
	box_frame.transform( &box_axis_x );
	box_frame.transform( &box_axis_y );
	box_frame.transform( &box_axis_z );

	// 直方体を構成する各三角面の頂点情報
	static const int  box_tri_vertices[ 12 ][ 3 ][ 3 ] = {
		{ { 0, 0, 0, }, { 0, 1, 0, }, { 1, 0, 0, } },
		{ { 1, 1, 0, }, { 1, 0, 0, }, { 0, 1, 0, } },
		{ { 0, 0, 0, }, { 0, 0, 1, }, { 0, 1, 0, } },
		{ { 0, 1, 1, }, { 0, 1, 0, }, { 0, 0, 1, } },
		{ { 0, 0, 0, }, { 1, 0, 0, }, { 0, 0, 1, } },
		{ { 1, 0, 1, }, { 0, 0, 1, }, { 1, 0, 0, } },
		{ { 0, 0, 1, }, { 1, 0, 1, }, { 0, 1, 1, } },
		{ { 1, 1, 1, }, { 0, 1, 1, }, { 1, 0, 1, } },
		{ { 1, 0, 0, }, { 1, 1, 0, }, { 1, 0, 1, } },
		{ { 1, 1, 1, }, { 1, 0, 1, }, { 1, 1, 0, } },
		{ { 0, 1, 0, }, { 0, 1, 1, }, { 1, 1, 0, } },
		{ { 1, 1, 1, }, { 1, 1, 0, }, { 0, 1, 1, } } };

	// 直方体の各面との交差判定を繰り返して、最も半直線の原点に近い交点を探索
	Point3f  v[ 3 ];
	Point3f  pos;
	Vector3f  vec;
	float  dist, min_dist = -1.0f;
	bool  cross_flag = false, tri_cross_flag;
	for ( int i=0; i<12; i++ )
	{
		for ( int j=0; j<3; j++ )
		{
			pos = box_org;
			if ( box_tri_vertices[ i ][ j ][ 0 ] > 0 )
				pos.add( box_axis_x );
			if ( box_tri_vertices[ i ][ j ][ 1 ] > 0 )
				pos.add( box_axis_y );
			if ( box_tri_vertices[ i ][ j ][ 2 ] > 0 )
				pos.add( box_axis_z );
			v[ j ] = pos;
		}

		// 半直線と三角面の交差判定
//		tri_cross_flag = RayTriCross( org, dir, v[0], v[1], v[2], pos, -1.0f );
		tri_cross_flag = RayTriCross( org, dir, v[0], v[1], v[2], pos );

		// 交差する場合は、最も近い交点を記録
		if ( tri_cross_flag )
		{
			vec.sub( pos, org );
			dist = vec.length();
			if ( !cross_flag || ( dist < min_dist ) )
			{
				cross = pos;
				min_dist = dist;
				cross_flag = true;
			}
		}
	}

	return  cross_flag;
}



//////////////////////////////////////////////////////////////////////////////
//
//  描画の補助処理（クラス関数）
//


//
//  線分の描画
//
void  ObjectLayout::DrawSegment( const Point3f & p0, const Point3f & p1, const Color3f & color )
{
	glBegin( GL_LINES );
		glColor3f( color.x, color.y, color.z );
		glVertex3f( p0.x, p0.y, p0.z );
		glVertex3f( p1.x, p1.y, p1.z );
	glEnd();
}


//
//  円弧の描画
//
void  ObjectLayout::DrawArc( const Point3f & v0, const Point3f & v1, const Color3f & color, int num_points )
{
	Vector3f  vec, axis, n0, n1;
	float  angle;
	AxisAngle4f  rot;
	Matrix3f  mat;

	// 円弧の開始ベクトル、終了ベクトル、軸ベクトル、回転角度、1回分の回転行列を計算
	n0.normalize( v0 );
	n1.normalize( v1 );
	axis.cross( n0, n1 );
	angle = acos( n0.dot( n1 ) );
	vec = v0;
	rot.set( axis, angle / ( num_points - 1 ) );
	mat.set( rot );

	// 円弧を折れ線で描画
	glBegin( GL_LINE_STRIP );
		glColor3f( color.x, color.y, color.z );
		for ( int i = 0; i < num_points; i++ )
		{
			glVertex3f( vec.x, vec.y, vec.z );
			mat.transform( &vec );
		}
	glEnd();
}


//
//  直方体の描画
//
void  ObjectLayout::DrawBox( const Matrix4f & frame, const Point3f & min_pos, const Point3f & max_pos, const Color3f & color )
{
	// OpenGLに渡す変換行列を計算
	Matrix4f  mat;
	mat.transpose( frame );

	// 中心点・大きさを計算
	Vector3f  center, size;
	center.add( min_pos, max_pos );
	center.scale( 0.5f );
	size.sub( max_pos, min_pos );

	// 直方体を描画
	glPushMatrix();
		glMultMatrixf( &mat.m00 );
		glTranslatef( center.x, center.y, center.z );
		glScalef( size.x, size.y, size.z );
		glColor3f( color.x, color.y, color.z );
		glLineWidth( 1.0f );
		glutWireCube( 1.0f );
	glPopMatrix();
}



// End of ObjectLayout.cpp
