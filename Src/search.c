/*
 * search.c
 *
 *  Created on: May 2, 2019
 *      Author: blue
 */

#include "global.h"


//+++++++++++++++++++++++++++++++++++++++++++++++
//search_init
//	探索系の変数とマップの初期化をする
//	引数：なし
//	戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void Search_Init(void){

  //	マウスフラグの初期化
  MF.FLAGS = 0;         //	フラグクリア

  //	探索系の変数の初期化====
  goal_x = GOAL_X;      //GOAL_Xはglobal.hにマクロ定義あり
  goal_y = GOAL_Y;      //GOAL_Yはglobal.hにマクロ定義あり
  map_Init();           //	マップの初期化
  mouse.x = 0;
  mouse.y = 0;          //	現在地の初期化
  mouse.dir = 0;        //	マウスの向きの初期化
  known_counter = 0;
}


/*-----------------------------------------------------------
		足立法探索走行（スラローム連続走行）
-----------------------------------------------------------*/
void Maze_Search_Slalom(motion *trans,motion *trans_known, motion *rot){

	if(MF.FLAG.SCND){
		load_map_from_eeprom();//2次走行ならマップを読み込む
	}

	//	スタート位置壁情報取得
										//	壁情報の初期化, 後壁はなくなる
	if(!MF.FLAG.SCND){
	get_wall_info();										//	壁情報の初期化, 後壁はなくなる
	wall_info &= ~0x88;										//	前壁は存在するはずがないので削除する
	write_map();											//	壁情報を地図に記入
	}
	//	前に壁が無い想定で問答無用で前進

	printf("Start Maze Run!\r\n");
	Adjust_Forward( trans->acceleration, trans->velocity );
	adv_pos();
	if(!MF.FLAG.SCND){
	get_wall_info();
	}
	//	歩数マップ・経路作成
	write_map();											//	壁情報を地図に記入
	r_cnt = 0;												//	経路カウンタの初期化
	ketsuate_cnt = 0;
	make_smap();											//	歩数マップ作成
	make_route();											//	最短経路探索（route配列に動作が格納される）



	//	探索走行
	do{
		//	進行
		switch(route[r_cnt++]){								//route配列によって進行を決定。経路カウンタを進める
			//	前進
			case 0x88:
				if(MF.FLAG.KNOWN == 1)
			{
				if(((route[r_cnt-1] & route[r_cnt]) == 0x88) && (route[r_cnt] != 0xff) &&(known_counter == 0) ){
						Accel_Known_Section(trans_known->acceleration,trans->velocity,trans_known->velocity);
						known_counter = 1;
					}
				else if((route[r_cnt] & 0x55) && (known_counter == 1)){
						Decel_Known_Section(trans_known->acceleration,trans_known->velocity,trans->velocity);
						known_counter = 0;
					}else{
						Forward_One_Section(trans_known->velocity);
				}
			}	else {
						Forward_One_Section(trans->velocity);
				}

				if(!MF.FLAG.SCND)	get_wall_info();
				break;

			//	Right Turn
			case 0x44:
				Buzzer_Scale(2000, 100);
				printf("turnR\r\n");
				Slalom_R(rot->velocity);
				if(!MF.FLAG.SCND)	get_wall_info();
				turn_dir(DIR_TURN_R90);
				break;

			//	180回転
			case 0x22:
				printf("turn180\r\n");
				Buzzer_Scale(5000, 100);
				Decel_Half_Section(trans->acceleration,trans->velocity);
				turn_dir(DIR_TURN_180);
				if(wall_info & 0x88){
						PivotL180(rot->acceleration,rot->velocity);
						Adjust_Back();
				}
//				Accel_Half_Section(trans->acceleration, trans->velocity);
				Adjust_Forward( trans->acceleration, trans->velocity );
				if(!MF.FLAG.SCND)	get_wall_info();
				break;

			//	左折
			case 0x11:
				Buzzer_Scale(1500, 100);

				printf("turnL\r\n");
				Slalom_L(rot->velocity);
				if(!MF.FLAG.SCND)	get_wall_info();
				turn_dir(DIR_TURN_L90);

				break;
		}
		adv_pos();

		if(!MF.FLAG.SCND)	conf_route();

	}while((mouse.x != goal_x) || (mouse.y != goal_y));

	Decel_Half_Section(trans->acceleration,trans->velocity);

	printf("Decel\r\n");
	Motor_Drive_Stop();
	printf("Goal\r\n");
	HAL_Delay(2000);

	PivotL180(rot->acceleration,rot->velocity);
	turn_dir(DIR_TURN_180);

	if(wall_info & 0x88){
		Adjust_Back();
	}
	Motor_Drive_Stop();

	if(!MF.FLAG.SCND){
		store_map_in_eeprom();
	}

}

//+++++++++++++++++++++++++++++++++++++++++++++++
//adv_pos
//	マイクロマウス内部位置情報で前進させる
//	引数：なし
//	戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void adv_pos(){

  switch(mouse.dir){                //	マイクロマウスが現在向いている方向で判定
  case 0x00:                        //	北方向に向いている場合
    mouse.y++;                      //	Y座標をインクリメント
    break;
  case 0x01:                        //	東方向に向いている場合
    mouse.x++;                      //	X座標をインクリメント
    break;
  case 0x02:                        //	南方向に向いている場合
    mouse.y--;                      //	Y座標をデクリメント
    break;
  case 0x03:                        //	西方向に向いている場合
    mouse.x--;                      //	X座標をデクリメント
    break;
  }
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//conf_route
//	進路を判定する
//	引数：なし
//	戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void conf_route(){

  //	壁情報書き込み-
  write_map();

  //	最短経路上に壁があれば進路変更
  if(wall_info & route[r_cnt]){
    make_smap();                    //	歩数マップを更新
    make_route();                   //	最短経路を更新
    r_cnt = 0;                      //	経路カウンタを0に
  }
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//map_Init
//	マップ格納配列map[][]の初期化をする
//	引数：なし
//	戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void map_Init(){

  //	変数宣言
  uint8_t x, y;                     //for文用変数

  //	初期化開始
  //	マップのクリア
  for(y = 0; y < 16; y++){          //	各Y座標で実行
    for(x = 0; x < 16; x++){        //	各X座標で実行
      map[y][x] = 0xf0;             //	上位4ビット（2次走行時）を壁あり，下位4ビット（1次走行時）を壁なしとする。
    }
  }

  //	確定壁の配置
  for(y = 0; y < 16; y++){          //	各Y座標で実行
    map[y][0] |= 0xf1;              //	最西に壁を配置
    map[y][15] |= 0xf4;             //	最東に壁を配置
  }
  for(x = 0; x < 16; x++){          //	各X座標で実行
    map[0][x] |= 0xf2;              //	最南に壁を配置
    map[15][x] |= 0xf8;             //	最北に壁を配置
  }
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//write_map
//	マップデータを書き込む
//	引数：なし
//	戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void write_map(){

  //	変数宣言
  uint8_t m_temp;                               //	向きを補正した壁情報

  //	壁情報の補正格納
  m_temp = (wall_info >> mouse.dir) & 0x0f;     //	センサ壁情報をmouse.dirで向きを補正させて下位4bit分を残す
  m_temp |= (m_temp << 4);                      //	上位4bitに下位4bitをコピー。この作業でm_tempにNESW順で壁が格納

  //	データの書き込み
  map[mouse.y][mouse.x] = m_temp;               //	現在地に壁情報書き込み
  //	周辺に書き込む
  //	北側について
  if(mouse.y != 15){                            //	現在最北端でないとき
    if(m_temp & 0x88){                          //	北壁がある場合
      map[mouse.y + 1][mouse.x] |= 0x22;        //	北側の区画から見て南壁ありを書き込む
    }else{                                      //	北壁がない場合
      map[mouse.y + 1][mouse.x] &= 0xDD;        //	北側の区画から見て南壁なしを書き込む
    }
  }
  //	東側について
  if(mouse.x != 15){                            //	現在最東端でないとき
    if(m_temp & 0x44){                          //	東壁がある場合
      map[mouse.y][mouse.x + 1] |= 0x11;        //	東側の区画から見て西壁ありを書き込む
    }else{                                      //	北壁がない場合
      map[mouse.y][mouse.x + 1] &= 0xEE;        //	東側の区画から見て西壁なしを書き込む
    }
  }
  //	南壁について
  if(mouse.y != 0){                             //	現在最南端でないとき
    if(m_temp & 0x22){                          //	南壁がある場合
      map[mouse.y - 1][mouse.x] |= 0x88;        //	南側の区画から見て北壁ありを書き込む
    }else{                                      //	南壁がない場合
      map[mouse.y - 1][mouse.x] &= 0x77;        //	南側の区画から見て北壁なしを書き込む
    }
  }
  //	西側について
  if(mouse.x != 0){                             //	現在最西端でないとき
    if(m_temp & 0x11){                          //	西壁がある場合
      map[mouse.y][mouse.x - 1] |= 0x44;        //	西側の区画から見て東壁ありを書き込む
    }else{                                      //	西壁がない場合
      map[mouse.y][mouse.x - 1] &= 0xBB;        //	西側の区画から見て東側なしを書き込む
    }
  }
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//write_map
//	マウスの方向を変更する
//	引数1：t_pat …… 回転方向(drive.hでマクロ定義)
//	戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void turn_dir(uint8_t t_pat){

  //	方向を変更
	mouse.dir = (mouse.dir + t_pat) & 0x03;       //	指定された分mouse.dirを回転させる
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//make_smap
// 歩数マップを作成する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void make_smap(void){

  //====変数宣言====
  uint8_t x, y;                                 //for文用変数

  //====歩数マップのクリア====
  for(y = 0; y <= 15; y++){                     //	各Y座標で実行
    for( x = 0; x <= 15; x++){                  //	各X座標で実行
      smap[y][x] = 0xff;                        //	未記入部分は歩数最大とする
    }
  }

  //====ゴール座標を0にする====
  uint8_t m_step = 0;                           //	歩数カウンタを0にする
  smap[goal_y][goal_x] = 0;

  //====自分の座標にたどり着くまでループ====
  do{
    //----マップ全域を捜索----
    for( y = 0; y <= 15; y++){                  //	各Y座標で実行
      for( x = 0; x <= 15; x++){                //	各X座標で実行
        //----現在最大の歩数を発見したとき----
        if( smap[y][x] == m_step){              //	歩数カウンタm_stepの値が現在最大の歩数
          uint8_t m_temp = map[y][x];           //	map配列からマップデータを取り出す
          if(MF.FLAG.SCND){                     //	二次走行用のマップを作成する場合（二次走行時はMF.FLAG.SCNDが立っている）
            m_temp >>= 4;                       //	上位4bitを使うので4bit分右にシフトさせる
          }
          //----北壁についての処理----
          if(!(m_temp & 0x08) && y != 15){      //	北壁がなく現在最北端でないとき
            if(smap[y+1][x] == 0xff){           //	北側が未記入なら
              smap[y+1][x] = m_step + 1;        //	次の歩数を書き込む
            }
          }
          //----東壁についての処理----
          if(!(m_temp & 0x04) && x != 15){      //	東壁がなく現在最東端でないとき
            if(smap[y][x+1] == 0xff){           //	東側が未記入なら
              smap[y][x+1] = m_step + 1;        //	次の歩数を書き込む
            }
          }
          //----南壁についての処理----
          if(!(m_temp & 0x02) && y != 0){       //	南壁がなく現在最南端でないとき
            if(smap[y-1][x] == 0xff){           //	南側が未記入なら
              smap[y-1][x] = m_step + 1;        //	次の歩数を書き込む
            }
          }
          //----西壁についての処理----
          if(!(m_temp & 0x01) && x != 0){       //	西壁がなく現在最西端でないとき
            if(smap[y][x-1] == 0xff){           //	西側が未記入なら
              smap[y][x-1] = m_step + 1;        //	次の歩数を書き込む
            }
          }
        }
      }
    }
    //====	歩数カウンタのインクリメント====
    m_step++;
  }while(smap[mouse.y][mouse.x] == 0xff);       //	現在座標が未記入ではなくなるまで実行
}



//+++++++++++++++++++++++++++++++++++++++++++++++
//make_route
//	最短経路を導出する
//	引数：なし
//	戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void make_route(){

  //	変数宣言
  uint8_t x, y;                                 //	X，Y座標
  uint8_t dir_temp =  mouse.dir;                //	マウスの方角を表すmouse.dirの値をdir_temp変数に退避させる

  //	最短経路を初期化
  uint16_t i;
  for(i = 0; i < 256; i++){
    route[i] = 0xff;                            //routeを0xffで初期化
  }

  //	歩数カウンタをセット
  uint8_t m_step = smap[mouse.y][mouse.x];      //	現在座標の歩数マップ値を取得

  //x, yに現在座標を書き込み
  x = mouse.x;
  y = mouse.y;

  //	最短経路を導出
  i = 0;
  do{
    uint8_t m_temp = map[y][x];                 //	比較用マップ情報の格納
    if(MF.FLAG.SCND){                           //	二次走行用のマップを作成する場合（二次走行時はMF.FLAG.SCNDが立っている）
      m_temp >>= 4;                             //	上位4bitを使うので4bit分右にシフトさせる
    }

    //----北を見る----
    if(!(m_temp & 0x08) && (smap[y+1][x] < m_step)){        //	北側に壁が無く、現在地より小さい歩数マップ値であれば
      route[i] = (0x00 - mouse.dir) & 0x03;                 //	route配列に進行方向を記録
      m_step = smap[y+1][x];                                //	最大歩数マップ値を更新
      y++;                                                  //	北に進んだのでY座標をインクリメント
    }
    //----東を見る----
    else if(!(m_temp & 0x04) && (smap[y][x+1] < m_step)){   //	東側に壁が無く、現在地より小さい歩数マップ値であれば
      route[i] = (0x01 - mouse.dir) & 0x03;                 //	route配列に進行方向を記録
      m_step = smap[y][x+1];                                //	最大歩数マップ値を更新
      x++;                                                  //	東に進んだのでX座標をインクリメント
    }
    //----南を見る----
    else if(!(m_temp & 0x02) && (smap[y-1][x] < m_step)){   //	南側に壁が無く、現在地より小さい歩数マップ値であれば
      route[i] = (0x02 - mouse.dir) & 0x03;                 //	route配列に進行方向を記録
      m_step = smap[y-1][x];                                //	最大歩数マップ値を更新
      y--;                                                  //	南に進んだのでY座標をデクリメント
    }
    //----西を見る----
    else if(!(m_temp & 0x01) && (smap[y][x-1] < m_step)){   //	西側に壁が無く、現在地より小さい歩数マップ値であれば
      route[i] = (0x03 - mouse.dir) & 0x03;                 //	route配列に進行方向を記録
      m_step = smap[y][x-1];                                //	最大歩数マップ値を更新
      x--;                                                  //	西に進んだのでX座標をデクリメント
    }

    //----格納データ形式変更----
    switch(route[i]){                 //	route配列に格納した要素値で分岐
    case 0x00:                        //	前進する場合
      route[i] = 0x88;                //	格納データ形式を変更
      break;
    case 0x01:                        //	右折する場合
      turn_dir(DIR_TURN_R90);         //	内部情報の方向を90度右回転
      route[i] = 0x44;                //	格納データ形式を変更
      break;
    case 0x02:                        //	Uターンする場合
      turn_dir(DIR_TURN_180);         //	内部情報の方向を180度回転
      route[i] = 0x22;                //	格納データ形式を変更
      break;
    case 0x03:                        //	左折する場合
      turn_dir(DIR_TURN_L90);         //	内部情報の方向を90度右回転
      route[i] = 0x11;                //	格納データ形式を変更
      break;
    default:                          //	それ以外の場合
      route[i] = 0x00;                //	格納データ形式を変更
      break;
    }
    i++;                              //	カウンタをインクリメント
  }while( smap[y][x] != 0);           //	進んだ先の歩数マップ値が0（=ゴール）になるまで実行
  mouse.dir = dir_temp;               //	dir_tempに退避させた値をmouse.dirにリストア
}




//+++++++++++++++++++++++++++++++++++++++++++++++
//store_map_in_eeprom
//	mapデータをeepromに格納する
//	引数：なし
//	戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void store_map_in_eeprom(void){
  eeprom_enable_write();
  int i;
  for(i = 0; i < 16; i++){
    int j;
    for(j = 0; j < 16; j++){
     eeprom_write_halfword(i*16 + j, (uint16_t) map[i][j]);
    }
  }
  eeprom_disable_write();
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//load_map_in_eeprom
// mapデータをeepromから取得する
//	引数：なし
//	戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void load_map_from_eeprom(void){
  int i;
  for(i = 0; i < 16; i++){
    int j;
    for(j = 0; j < 16; j++){
      map[i][j] = (uint8_t) eeprom_read_halfword(i*16 + j);
    }
  }
}
