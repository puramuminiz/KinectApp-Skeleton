#include <iostream>
#include <sstream>
#include <vector>
#include <algorithm>

// NuiApi.hの前にWindows.hをインクルードする
#include <Windows.h>
#include <NuiApi.h>

#include <opencv2/opencv.hpp>

#define ERROR_CHECK( ret )			\
	if(ret != S_OK){				\
		std::stringstream ss;		\
		ss << "failed" #ret " " << std::hex << ret << std::endl;	\
		throw std::runtime_error( ss.str().c_str() );				\
	}
#define DEPTH_RANGE_MIN 800		//認識できる距離の最小値
#define DEPTH_RANGE_MAX 4000	//認識できる距離の最大値
#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480

#define VALUE_MAX 255

	const NUI_IMAGE_RESOLUTION CAMERA_RESOLUTION = NUI_IMAGE_RESOLUTION_640x480;

	class KinectSample
	{
	private:

		INuiSensor* kinect;
		HANDLE imageStreamHandle;
		HANDLE depthStreamHandle;
		HANDLE streamEvent;


		DWORD width;
		DWORD height;

	public:

		KinectSample()
		{
		}

		~KinectSample()
		{
			// 終了処理
			if(kinect != 0){
				kinect->NuiShutdown();
				kinect->Release();
			}
		}

		void initialize()
		{
			createInstance();

			// Kinectの設定を初期化する
			ERROR_CHECK( kinect->NuiInitialize( NUI_INITIALIZE_FLAG_USES_COLOR |
												NUI_INITIALIZE_FLAG_USES_DEPTH |
												NUI_INITIALIZE_FLAG_USES_SKELETON ) );


			// 距離カメラを初期化する
			ERROR_CHECK( kinect->NuiImageStreamOpen( NUI_IMAGE_TYPE_DEPTH,
													 CAMERA_RESOLUTION,
													 0, 2, 0, &depthStreamHandle ) );

			//スケルトンの初期化
			ERROR_CHECK( kinect->NuiSkeletonTrackingEnable( 0, 0 ) );

			// フレーム更新イベントのハンドルを作成する
			streamEvent = ::CreateEvent( 0, TRUE, FALSE, 0 );
			ERROR_CHECK( kinect->NuiSetFrameEndEvent( streamEvent, 0 ) );

			// 指定した解像度の画面サイズを取得する
			::NuiImageResolutionToSize(CAMERA_RESOLUTION, width, height );
		}

		void run()
		{
			// 取得用
			//cv::Mat sensor( 480, 640, CV_16UC1);
			// 距離画像用
			cv::Mat depthimg( IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1);
			// 閾値処理用
			cv::Mat binaryimg( IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1);
			// スケルトン用
			cv::Mat skeletonimg(  IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);

			// メインループ
			while( 1 ){
				// データの更新を待つ
				DWORD ret = ::WaitForSingleObject( streamEvent, INFINITE );
				::ResetEvent( streamEvent );

				// 距離画像の生成
				//drawDepthImage( depthimg );
				drawSkeleton( skeletonimg );

				// 閾値処理
				//cv::threshold( depthimg, binaryimg, 150, 255, CV_THRESH_BINARY);
								
				// 画像を表示する

				//cv::imshow( "depthimg", depthimg );
				//cv::imshow( "binaryimg", binaryimg );
				cv::imshow( "skeletonimg", skeletonimg );


				// 終了のためのキー入力チェック
				int key = cv::waitKey(10);
				if ( key == 'q' ){
					break;
				}
			}
		}

	private:


		void createInstance()
		{
			// 接続されているKinectの数を取得する
			int count = 0;
			ERROR_CHECK( ::NuiGetSensorCount( &count ) );
			if ( count == 0 ){
				throw std::runtime_error( "Kinectを接続してください" );
			}

			// 最初のKinectのインスタンスを作成する
			ERROR_CHECK( ::NuiCreateSensorByIndex( 0, &kinect ) );

			// Kinectの状態を取得する
			HRESULT status = kinect->NuiStatus();
			if ( status != S_OK ){
				throw std::runtime_error( "kinectが利用可能ではありません" );
			}
		}



		void drawDepthImage( cv::Mat& depthimg )
		{			
			cv::Mat sensor( 480, 640, CV_16UC1 );

			// 距離カメラのフレームデータを取得する
			NUI_IMAGE_FRAME depthFrame = { 0 };
			ERROR_CHECK( kinect->NuiImageStreamGetNextFrame( depthStreamHandle, 0,
															 &depthFrame ) );

			// 距離データを取得する
			NUI_LOCKED_RECT depthData = { 0 };
			depthFrame.pFrameTexture->LockRect( 0, &depthData, 0, 0 );

			USHORT* depth = (USHORT*)depthData.pBits;

			for(int y = 0; y < depthimg.rows; y++){
				for(int x = 0; x < depthimg.cols; x++){

				// 画像のチャネル数分だけループ。白黒の場合は1回、カラーの場合は3回　　　　
					USHORT distance = ::NuiDepthPixelToDepth( depth[ y * depthimg.step + x * depthimg.elemSize()] );
					if( distance < DEPTH_RANGE_MIN || distance > DEPTH_RANGE_MAX ) distance = 4000;				
					USHORT val = VALUE_MAX - VALUE_MAX * ( distance - DEPTH_RANGE_MIN ) / ( DEPTH_RANGE_MAX - DEPTH_RANGE_MIN );
					depthimg.data[ y * depthimg.step + x * depthimg.elemSize() ] = val;
				}
			}

		// フレームデータを開放する
			ERROR_CHECK( kinect->NuiImageStreamReleaseFrame( depthStreamHandle,
															 &depthFrame ) );
		}

		void drawSkeleton( cv::Mat& image )
		{
			// 前フレームのスケルトンの描画を削除
			for(int y = 0; y < image.rows; y++){
				for(int x = 0; x < image.cols; x++){
					for(int c = 0; c < image.channels(); c++){
						image.data[ y * image.step + x * image.elemSize() + c ] = 0;
					}
				}
			}
			// スケルトンのフレームを取得する
			NUI_SKELETON_FRAME skeletonFrame = { 0 };
			kinect->NuiSkeletonGetNextFrame( 0, &skeletonFrame );

			for( int i = 0; i < NUI_SKELETON_COUNT; ++i ){
				NUI_SKELETON_DATA& skeletonData = skeletonFrame.SkeletonData[i];
				// スケルトンが追跡されている状態
				if ( skeletonData.eTrackingState == NUI_SKELETON_TRACKED ){
					for( int j = 0; j < NUI_SKELETON_POSITION_COUNT; ++j ){
						// ジョイントが追跡していない状態以外(追跡、推定)の場合
						if( skeletonData.eSkeletonPositionTrackingState[j] != NUI_SKELETON_POSITION_NOT_TRACKED ){
							// ジョイントの座標を描画する
							drawJoint( image, skeletonData.SkeletonPositions[j] );
						}
					}
				}
				// プレイヤーの位置のみ追跡している状態
				else if( skeletonData.eTrackingState == NUI_SKELETON_POSITION_ONLY ){
					// プレイヤーの位置を描画する
					drawJoint( image, skeletonData.Position );
				}
			}
		}

		void drawJoint( cv::Mat& image, Vector4 position )
		{
			FLOAT depthX = 0, depthY = 0;
			::NuiTransformSkeletonToDepthImage( position, &depthX, &depthY,
												CAMERA_RESOLUTION );

			LONG colorX = 0;
			LONG colorY = 0;

			kinect->NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(
												CAMERA_RESOLUTION, CAMERA_RESOLUTION, 0, (LONG)depthX, (LONG)depthY,
												0, &colorX, &colorY );

			cv::circle( image, cv::Point(colorX, colorY ), 10,
							   cv::Scalar( 0, 255, 0 ), 5 );
		}
	};

	void main()
	{
		try{
			KinectSample kinect;
			kinect.initialize();
			kinect.run();
		}
		catch ( std::exception& ex ) {
			std::cout << ex.what() << std::endl;
		}
	}