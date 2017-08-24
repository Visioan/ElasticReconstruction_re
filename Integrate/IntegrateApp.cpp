
#include "IntegrateApp.h"

CIntegrateApp::CIntegrateApp(ImageGrabber & grabber, bool use_device)
	: cols_( 640 ), rows_( 480 )
	, volume_( cols_, rows_ )
	, grabber_( grabber )
	, use_device_( use_device )
	, exit_( false )
	, registration_( false )
	, time_ms_( 0 )
	, frame_id_( 0 )
	, traj_filename_( "" )
	, pose_filename_( "" )
	, seg_filename_( "" )
	, camera_filename_( "" )
	, ctr_filename_( "" )
	, pcd_filename_( "world.pcd" )
	, ctr_num_( 0 )
	, ctr_resolution_( 8 )
	, ctr_interval_( 50 )
	, ctr_length_( 3.0 )
	, start_from_( -1 )
	, end_at_( 100000000 )
{
	registration_ = true;
	cout << "Registration mode: " << ( registration_ ? "On" : "Off (not supported by source)" ) << endl;

	depth_.resize( cols_ * rows_ );
	scaled_depth_.resize( cols_ * rows_ );
	scale_ = 1.0;
#ifdef IMAGE_VIEWER
	viewer_depth_.setWindowTitle( "Depth stream" );
	viewer_depth_.setPosition( 0, 0 );
#endif
}

CIntegrateApp::~CIntegrateApp(void)
{
}

void CIntegrateApp::Init()
{
	if ( boost::filesystem::exists( camera_filename_ ) ) {
		volume_.camera_.LoadFromFile( camera_filename_ );
	}

	if ( ctr_num_ > 0 && boost::filesystem::exists( ctr_filename_ ) && boost::filesystem::exists( seg_filename_ ) ) {
		grids_.resize( ctr_num_ );
		FILE * f = fopen( ctr_filename_.c_str(), "r" );
		for ( int i = 0; i < ctr_num_; i++ ) {
			grids_[ i ].Load( f, ctr_resolution_, ctr_length_ );
		}
		fclose( f );
	} else {
		ctr_num_ = 0;
	}

	if ( boost::filesystem::exists( traj_filename_ ) ) {
		traj_.LoadFromFile( traj_filename_ );
	}

	if ( boost::filesystem::exists( seg_filename_ ) ) {
		seg_traj_.LoadFromFile( seg_filename_ );

		if ( boost::filesystem::exists( pose_filename_ ) ) {
			pose_traj_.LoadFromFile( pose_filename_ );
			traj_.data_.clear();
			for ( int i = 0; i < ( int )pose_traj_.data_.size(); i++ ) {
				for ( int j = 0; j < ctr_interval_; j++ ) {
					int idx = i * ctr_interval_ + j;
					if (idx >= seg_traj_.data_.size())
						break;
					traj_.data_.push_back( FramedTransformation( idx, idx, idx + 1, pose_traj_.data_[ i ].transformation_ * seg_traj_.data_[ idx ].transformation_ ) );
 				}
			}
			PCL_WARN( "Trajectory created from pose and segment trajectories.\n" );
		}
	}
}

void CIntegrateApp::StartMainLoop( bool triggered_capture )
{
	vector<string> vstrcolorImgs, vstrDepthImgs;
	vector<double> vdoubltTimestamps;
	grabber_.getColorDepthImgs(vstrcolorImgs, vstrDepthImgs, vdoubltTimestamps);
	int totalNum = vstrDepthImgs.size();

	{
		for (int i = 0; i < totalNum; i++)
		{
			//if (i == totalNum - 1)
			//	cin.get();
			bool has_data = true;
			frame_id_ ++;
			Mat2Pixel(vstrcolorImgs[i], vstrDepthImgs[i]);
			try {
				this->Execute( has_data );

#ifdef IMAGE_VIEWER
				viewer_depth_.showShortImage( &depth_[ 0 ], cols_, rows_, 0, 3000, true );
				viewer_depth_.spinOnce( 3 );
#endif
			}
			catch (const std::bad_alloc& /*e*/) { cout << "Bad alloc" << endl; break; }
			catch (const std::exception& /*e*/) { cout << "Exception" << endl; break; }
		}
		volume_.SaveWorld( pcd_filename_ );
		cout << "Total " << frame_id_ << " frames processed." << endl;
		string savetraj = "Traj.txt";
		traj_.SaveToFile(savetraj);
		cout << "Save Traj  " << savetraj << endl;
		//volume_.SaveWorld( std::string( "world.pcd" ) );
	}
}

//////////////////////////////////////////////
// Capture functions
//////////////////////////////////////////////
void CIntegrateApp::Mat2Pixel(const string & colorimgpath, const string & depthimgpath)
{

	//cv::Mat colorimg = cv::imread(colorimgpath, CV_LOAD_IMAGE_ANYCOLOR);
	cv::Mat depthimg = cv::imread(depthimgpath, CV_LOAD_IMAGE_ANYDEPTH);

	depthimg.convertTo(depthimg, CV_16UC1, (double)scale_);

	depth_.clear();
	scaled_depth_.clear();
	cols_ = depthimg.cols;
	rows_ = depthimg.rows;
	scaled_depth_.resize(cols_*rows_);;


	depth_.assign((unsigned short*)depthimg.datastart, (unsigned short*)depthimg.dataend);

}


void CIntegrateApp::Execute( bool has_data )
{
	if ( !has_data ) {
		return;
	}

	if ( traj_.data_[ frame_id_ - 1 ].frame_ == -1 ) {
		return;
	}

	//if ( frame_id_ >= traj_.data_.size() ) {
	//	exit_ = true;
	//	return;
	//}
	PCL_WARN("Frames processed : %d\n", frame_id_);
	if ( frame_id_ % 100 == 0 ) {
		PCL_WARN( "Frames processed : %d / %d\n", frame_id_, traj_.data_.size() );
	}

	if ( frame_id_ < start_from_ || frame_id_ > end_at_ ) {
		if ( frame_id_ > end_at_ ) {
			PCL_WARN( "Reaching the specified end point.\n" );
			exit_ = true;
		}
		return;
	}

	if ( ctr_num_ > 0 ) {
		Reproject();
		if ( exit_ ) {
			return;
		}
	}

	volume_.ScaleDepth( depth_, scaled_depth_ );
	volume_.Integrate( depth_, scaled_depth_, traj_.data_[ frame_id_ - 1 ].transformation_ );
}

void CIntegrateApp::Reproject()
{
	if ( frame_id_ > ctr_interval_ * ctr_num_ ) {
		exit_ = true;
		return;
	}

	depth_buffer_.resize( depth_.size() );
	for ( int i = 0; i < cols_ * rows_; i++ ) {
		depth_buffer_[ i ] = depth_[ i ];
		depth_[ i ] = 0;
	}

	int chunk = ( frame_id_ - 1 ) / ctr_interval_;
	Eigen::Matrix4d TiT0Ai_adj = traj_.data_[ frame_id_ - 1 ].transformation_.inverse() * traj_.data_[ 0 ].transformation_ * seg_traj_.data_[ 0 ].transformation_.inverse();

	int uu, vv;
	unsigned short dd;
	double x, y, z;
	for ( int v = 0; v < rows_; v += 1 ) {
		for ( int u = 0; u < cols_; u += 1 ) {
			unsigned short d = depth_buffer_[ v * cols_ + u ];
			if ( volume_.UVD2XYZ( u, v, d, x, y, z ) ) {
				Eigen::Vector4d dummy = seg_traj_.data_[ frame_id_ - 1 ].transformation_ * Eigen::Vector4d( x, y, z, 1 );
				Coordinate coo;
				Eigen::Vector3f pos;

				if ( grids_[ chunk ].GetCoordinate( Eigen::Vector3f( dummy( 0 ), dummy( 1 ), dummy( 2 ) ), coo ) ) {		// in the box, thus has the right coo
					grids_[ chunk ].GetPosition( coo, pos );
					Eigen::Vector4d reproj_pos = TiT0Ai_adj * Eigen::Vector4d( pos( 0 ), pos( 1 ), pos( 2 ), 1.0 );

					if ( volume_.XYZ2UVD( reproj_pos( 0 ), reproj_pos( 1 ), reproj_pos( 2 ), uu, vv, dd ) ) {
						unsigned short ddd = depth_[ vv * cols_ + uu ];
						if ( ddd == 0 || ddd > dd ) {
							depth_[ vv * cols_ + uu ] = dd;
						}
					}
				}
			}
		}
	}
}