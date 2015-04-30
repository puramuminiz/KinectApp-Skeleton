#include <iostream>
#include <sstream>
#include <vector>
#include <algorithm>

// NuiApi.h�̑O��Windows.h���C���N���[�h����
#include <Windows.h>
#include <NuiApi.h>

#include <opencv2/opencv.hpp>

#define ERROR_CHECK( ret )			\
	if(ret != S_OK){				\
		std::stringstream ss;		\
		ss << "failed" #ret " " << std::hex << ret << std::endl;	\
		throw std::runtime_error( ss.str().c_str() );				\
	}
#define DEPTH_RANGE_MIN 800		//�F���ł��鋗���̍ŏ��l
#define DEPTH_RANGE_MAX 4000	//�F���ł��鋗���̍ő�l
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
			// �I������
			if(kinect != 0){
				kinect->NuiShutdown();
				kinect->Release();
			}
		}

		void initialize()
		{
			createInstance();

			// Kinect�̐ݒ������������
			ERROR_CHECK( kinect->NuiInitialize( NUI_INITIALIZE_FLAG_USES_COLOR |
												NUI_INITIALIZE_FLAG_USES_DEPTH |
												NUI_INITIALIZE_FLAG_USES_SKELETON ) );


			// �����J����������������
			ERROR_CHECK( kinect->NuiImageStreamOpen( NUI_IMAGE_TYPE_DEPTH,
													 CAMERA_RESOLUTION,
													 0, 2, 0, &depthStreamHandle ) );

			//�X�P���g���̏�����
			ERROR_CHECK( kinect->NuiSkeletonTrackingEnable( 0, 0 ) );

			// �t���[���X�V�C�x���g�̃n���h�����쐬����
			streamEvent = ::CreateEvent( 0, TRUE, FALSE, 0 );
			ERROR_CHECK( kinect->NuiSetFrameEndEvent( streamEvent, 0 ) );

			// �w�肵���𑜓x�̉�ʃT�C�Y���擾����
			::NuiImageResolutionToSize(CAMERA_RESOLUTION, width, height );
		}

		void run()
		{
			// �擾�p
			//cv::Mat sensor( 480, 640, CV_16UC1);
			// �����摜�p
			cv::Mat depthimg( IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1);
			// 臒l�����p
			cv::Mat binaryimg( IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1);
			// �X�P���g���p
			cv::Mat skeletonimg(  IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);

			// ���C�����[�v
			while( 1 ){
				// �f�[�^�̍X�V��҂�
				DWORD ret = ::WaitForSingleObject( streamEvent, INFINITE );
				::ResetEvent( streamEvent );

				// �����摜�̐���
				//drawDepthImage( depthimg );
				drawSkeleton( skeletonimg );

				// 臒l����
				//cv::threshold( depthimg, binaryimg, 150, 255, CV_THRESH_BINARY);
								
				// �摜��\������

				//cv::imshow( "depthimg", depthimg );
				//cv::imshow( "binaryimg", binaryimg );
				cv::imshow( "skeletonimg", skeletonimg );


				// �I���̂��߂̃L�[���̓`�F�b�N
				int key = cv::waitKey(10);
				if ( key == 'q' ){
					break;
				}
			}
		}

	private:


		void createInstance()
		{
			// �ڑ�����Ă���Kinect�̐����擾����
			int count = 0;
			ERROR_CHECK( ::NuiGetSensorCount( &count ) );
			if ( count == 0 ){
				throw std::runtime_error( "Kinect��ڑ����Ă�������" );
			}

			// �ŏ���Kinect�̃C���X�^���X���쐬����
			ERROR_CHECK( ::NuiCreateSensorByIndex( 0, &kinect ) );

			// Kinect�̏�Ԃ��擾����
			HRESULT status = kinect->NuiStatus();
			if ( status != S_OK ){
				throw std::runtime_error( "kinect�����p�\�ł͂���܂���" );
			}
		}



		void drawDepthImage( cv::Mat& depthimg )
		{			
			cv::Mat sensor( 480, 640, CV_16UC1 );

			// �����J�����̃t���[���f�[�^���擾����
			NUI_IMAGE_FRAME depthFrame = { 0 };
			ERROR_CHECK( kinect->NuiImageStreamGetNextFrame( depthStreamHandle, 0,
															 &depthFrame ) );

			// �����f�[�^���擾����
			NUI_LOCKED_RECT depthData = { 0 };
			depthFrame.pFrameTexture->LockRect( 0, &depthData, 0, 0 );

			USHORT* depth = (USHORT*)depthData.pBits;

			for(int y = 0; y < depthimg.rows; y++){
				for(int x = 0; x < depthimg.cols; x++){

				// �摜�̃`���l�������������[�v�B�����̏ꍇ��1��A�J���[�̏ꍇ��3��@�@�@�@
					USHORT distance = ::NuiDepthPixelToDepth( depth[ y * depthimg.step + x * depthimg.elemSize()] );
					if( distance < DEPTH_RANGE_MIN || distance > DEPTH_RANGE_MAX ) distance = 4000;				
					USHORT val = VALUE_MAX - VALUE_MAX * ( distance - DEPTH_RANGE_MIN ) / ( DEPTH_RANGE_MAX - DEPTH_RANGE_MIN );
					depthimg.data[ y * depthimg.step + x * depthimg.elemSize() ] = val;
				}
			}

		// �t���[���f�[�^���J������
			ERROR_CHECK( kinect->NuiImageStreamReleaseFrame( depthStreamHandle,
															 &depthFrame ) );
		}

		void drawSkeleton( cv::Mat& image )
		{
			// �O�t���[���̃X�P���g���̕`����폜
			for(int y = 0; y < image.rows; y++){
				for(int x = 0; x < image.cols; x++){
					for(int c = 0; c < image.channels(); c++){
						image.data[ y * image.step + x * image.elemSize() + c ] = 0;
					}
				}
			}
			// �X�P���g���̃t���[�����擾����
			NUI_SKELETON_FRAME skeletonFrame = { 0 };
			kinect->NuiSkeletonGetNextFrame( 0, &skeletonFrame );

			for( int i = 0; i < NUI_SKELETON_COUNT; ++i ){
				NUI_SKELETON_DATA& skeletonData = skeletonFrame.SkeletonData[i];
				// �X�P���g�����ǐՂ���Ă�����
				if ( skeletonData.eTrackingState == NUI_SKELETON_TRACKED ){
					for( int j = 0; j < NUI_SKELETON_POSITION_COUNT; ++j ){
						// �W���C���g���ǐՂ��Ă��Ȃ���ԈȊO(�ǐՁA����)�̏ꍇ
						if( skeletonData.eSkeletonPositionTrackingState[j] != NUI_SKELETON_POSITION_NOT_TRACKED ){
							// �W���C���g�̍��W��`�悷��
							drawJoint( image, skeletonData.SkeletonPositions[j] );
						}
					}
				}
				// �v���C���[�̈ʒu�̂ݒǐՂ��Ă�����
				else if( skeletonData.eTrackingState == NUI_SKELETON_POSITION_ONLY ){
					// �v���C���[�̈ʒu��`�悷��
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