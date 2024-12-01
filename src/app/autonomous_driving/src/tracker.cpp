#include "tracker.hpp"

inline float euclideanDist(const Eigen::VectorXd& _p1, const Eigen::VectorXd& _p2){
		return sqrt((_p1(0) - _p2(0))* (_p1(0) - _p2(0)) + (_p1(1) - _p2(1)) * (_p1(1) - _p2(1)));
}

// prev_detections_ : 이전 프레임에서 탐지 객체
// not_associated_ : 현재 프레임에서 아직 추적되지 않은 탐지 객체



// 첫 번째 Init에서는 Detection -> prev_detections
// 두 번째 Init에서는 init 완료, tracking 시작 X. -> not_associated에 det을 넘기고 manage_track 시작

void Tracker::Init(const Detection& detections, float& time){

	if(!init_){
		//std::cout<<"#################### TRACKER INIT ####################"<<std::endl;
		prev_detections_.clear();
		prev_detections_.swap(prev_detections_);

		for(auto& det:detections){
			prev_detections_.push_back(det);
		}

		init_ = true;
    // init_: true , start_tracking : false 
	}else if(init_ && !start_tracking_){
		//std::cout<<"#################### TRACKER start_tracking_ INIT ####################"<<std::endl;
		not_associated_.clear();
		not_associated_.swap(not_associated_);

		for(auto& det:detections){
			not_associated_.push_back(det);
		}
        // 새로 tracks_ 생성
		manage_tracks(time);
        // tracks_ size가 0보다 크면 tracks_ 추적 객체가 하나라도 있으니 추적이 시작되었음을 알린다.
        // tracks_ size가 0보다 작으면 tracks_ 추적 객체가 한개도 없으니 not_associated를 prev_detections에 넘긴다.
		if(tracks_.size()>0){
			start_tracking_= true;
		}else{
			prev_detections_ = not_associated_;
		}
	}

	pretime = time;
}

void Tracker::manage_tracks(float& time){
	const uint& prevDetSize = prev_detections_.size();
	const uint& deteSize = not_associated_.size(); 
	
    // k-1번째 탐지가 안됐으면 k번째 탐지를 넘겨줌
	if(prevDetSize == 0){
		prev_detections_ = not_associated_; // 
    // k번째 탐지가 안됐으면 이전 탐지 객체들을 비우고 다음 프레임을 준비
	}else if(deteSize == 0){
		prev_detections_.clear();
		prev_detections_.swap(prev_detections_);
    // 추적 과정
    }else{
        //std::cout<<"######## manage track not asso ###############"<<not_associated_.size()<<std::endl;

        cv::Mat assigmentsBin = cv::Mat::zeros(cv::Size(deteSize, prevDetSize), CV_32SC1); // 매칭된 객체 1 , 매칭안된 객체 0
        cv::Mat costMat = cv::Mat(cv::Size(deteSize, prevDetSize), CV_32FC1); // 유클리디안 거리 기반 비용 행렬
        // IoU 관련 행렬을 제거

        std::vector<int> assignments;
        std::vector<float> costs(deteSize * prevDetSize);

        // 유클리디안 거리 계산 K-1 prev_detections와 not_associated_와의 거리
        for(uint i = 0; i < prevDetSize; ++i){
            for(uint j = 0; j < deteSize; ++j){
                costs.at(i + j * prevDetSize) = euclideanDist(not_associated_.at(j).position, prev_detections_.at(i).position);
                costMat.at<float>(i, j) = costs.at(i + j * prevDetSize); // 비용 행렬에 유클리디안 거리 저장
            }
        }

        // std::cout<<"########## costMat #####\n"<<costMat<<std::endl;
        // ------------------*************아직 해결 X *******************----------------------------
        AssignmentProblemSolver APS; // 헝가리안 알고리즘
        APS.Solve(costs, prevDetSize, deteSize, assignments, AssignmentProblemSolver::optimal);

        const uint& assSize = assignments.size();
        for(uint i = 0; i < assSize; ++i){
            if(assignments[i] != -1 && costMat.at<float>(i, assignments[i]) < dist_thresh_){ // 유클리디안 거리 임계값만 사용
                assigmentsBin.at<int>(i, assignments[i]) = 1;
            }
        }

        const uint& rows = assigmentsBin.rows;
        const uint& cols = assigmentsBin.cols;

        for(uint i = 0; i < rows; ++i){
            for(uint j = 0; j < cols; ++j){
                if(assigmentsBin.at<int>(i, j)){
                    // 속도 및 각도를 계산하여 추적 객체를 생성
                    float velo = not_associated_.at(j).velocity;  // 속도
                    float angle = not_associated_.at(j).yaw;  // 각도
                    // std::cout<<vx<<" "<<vy<<" "<<angle<<std::endl;
                    std::shared_ptr<Track> tr(new Track(track_id_, time, not_associated_.at(j), velo, angle)); // 추적 객체 생성
                    track_id_++;
                    tracks_.push_back(tr); // 추적 객체를 tracks_에 추가
                }
            }
        }
    

        //------------------------------------tracks_할당 완-------------------------------------------
       
    	cv::Mat notAssignedDet(cv::Size(assigmentsBin.cols, 1), CV_32SC1, cv::Scalar(0));
    	for(uint i = 0; i < assigmentsBin.rows; ++i){
      		notAssignedDet += assigmentsBin.row(i);
    	}
    
    	notAssignedDet.convertTo(notAssignedDet, CV_8UC1);
    	notAssignedDet = notAssignedDet == 0;
    
    	cv::Mat dets;
    	cv::findNonZero(notAssignedDet, dets);
    	prev_detections_.clear();
    	for(uint i = 0; i < dets.total(); ++i){
      		const uint& idx = dets.at<cv::Point>(i).x;
      		prev_detections_.push_back(not_associated_.at(i));
    	}

        // 매칭되지 않은 탐지 객체들 prev_detections에 저장하여 다음 프레임에서 매칭 시도
  	}
}


void Tracker::track(const Detection& detections,float& time, std::vector<Eigen::VectorXd>& result){
	
	if(!init_ || !start_tracking_){
		Init(detections,time); // Init에서 이미 tracks_ 생성해서 옴.
	}else{

		//std::cout<<"#################### TRACKER tracking #####################"<<std::endl;
		not_associated_.clear();
		not_associated_.swap(not_associated_);
		
		//std::cout<<"tracksstart "<<tracks_.size()<<" confirmed_tracks_ "<<confirmed_tracks_.size()<<std::endl;
        

        // tracks_의 confimed이 "새로" 됐다면 confirmed_tracks_에 할당! 아니면 unconfirmed_tracks_에 할당
		for (auto& track:tracks_){
			if(track->GetTrackState()==Track_state_.Confirmed && track->Age()==0){
				confirmed_tracks_.push_back(track);
			}else{
				unconfirmed_tracks_.push_back(track);
			}
		}


        // tracks_를 confirmed와 unconfirmed에 나눠서 할당했으니 tracks_는 다시 초기화하고 메모리 비우기!
		//std::cout<<"confirmed_tracks_ "<<confirmed_tracks_.size()<<" "<<"unconfirmed_tracks_ "<<unconfirmed_tracks_.size()<<std::endl;
		tracks_.clear();
		tracks_.swap(tracks_);

        // Measurement가 추적된 객체 tracks_와 Associated 를 말하는 조건
		std::vector<bool> MeaisAsso(detections.size(),false);


        // 연관 매트릭스
		cv::Mat_<int> q(cv::Size(confirmed_tracks_.size(), detections.size()), int(0));
		

    	Detection selected_detections;

        // 각 confirmed_tracks는 Prediction 함수를 호출하여 주어진 time에 대한 예측 수행
		for(const auto& track:confirmed_tracks_){
			track->Prediction(time);
		}

		std::vector<track_ptr> prun_tracks;
    		associate(selected_detections, q, detections);

		if(q.total()==0){
			for(auto& track:confirmed_tracks_){
				track->MarkMissed();
			}
		}else{
			std::vector<std::vector<int> > cltrack;
			std::vector<std::vector<int> > cldet;
			std::vector<bool> missed_tracks(confirmed_tracks_.size(),true);

			clusterq(q, cltrack, cldet);
			int clustersize = cltrack.size();
			for(int i =0; i<clustersize; ++i){
				int colsize = cltrack[i].size();
				int rowsize = cldet[i].size();
				//std::cout<<"###############q \n"<<q<<"\n"<<std::endl;
				//std::cout<<"colsize "<<colsize <<" rowsize "<<rowsize<<std::endl;

				cv::Mat x = cv::Mat_<int>(cv::Size(colsize + 1, rowsize), int(1));

				//std::cout<<"########## x ##########\n"<<x<<std::endl;
				//std::cout<<"########## test ##########\n"<<q.at<int>(0, 3)<<std::endl;

				for(int row=0; row<rowsize; ++row){
					int rowinq = cldet[i][row];
					for(int col=1; col<x.cols; ++col){
						//std::cout<<"col "<<cltrack[i][col-1]<<std::endl;
						//std::cout<<"row "<<rowinq<<" "<<std::endl;
						//std::cout<<q.at<int>(rowinq,cltrack[i][col-1])<<std::endl;
						x.at<int>(row,col) = q.at<int>(rowinq, cltrack[i][col-1]);
					}
				}
				//q(cv::Rect(colmin, rowmin, colsize, rowsize)).copyTo(x(cv::Rect(1, 0, colsize, rowsize)));
				//std::cout<<"########## q ##########\n"<<q(cv::Rect(colmin, rowmin, colsize, rowsize)).clone()<<std::endl;
				//std::cout<<"########## x ##########\n"<<x(cv::Rect(1, 0, colsize, rowsize))<<std::endl;

				//std::cout<<"########## x ##########\n"<<x<<std::endl;
				std::vector<track_ptr> tracks;
				for(auto trackid:cltrack[i]){
					//std::cout<<"trackid "<<trackid-1<<std::endl;
					missed_tracks[trackid-1] = false;
					tracks.push_back(confirmed_tracks_[trackid-1]);
				}
				Detection selectdets;
				for(auto detid:cldet[i]){
					//std::cout<<"detid "<<detid<<std::endl;
					selectdets.push_back(selected_detections[detid]);
				}
				const Matrices& association_matrices = generate_hypothesis(x);//生成假设矩阵
				//std::cout<<"########## association_matrices ##########\n"<<std::endl;

				Eigen::MatrixXd beta = joint_probability(association_matrices, selectdets, tracks);//JPDAF
				//std::cout<<"########## BETA part ##########\n"<<beta<<std::endl;
				int b=0;
				for(const auto& track:tracks){
					std::vector<Eigen::VectorXd> Zv;
					for(auto det : selectdets){
						Zv.push_back(det.position);
					}
					track->Update(Zv, beta.col(b), beta(beta.rows() - 1, b), time);//TODO change the imm_ukf
					b++;
					tracks_.push_back(track);
					prun_tracks.push_back(track);
				}
			}

			for(int i=0; i<confirmed_tracks_.size(); ++i){
				if(missed_tracks[i]){
					confirmed_tracks_[i]->MarkMissed();
					tracks_.push_back(confirmed_tracks_[i]);
				}
			}
		}


    		/*std::cout<<"q \n "<<q<<"\n q.total() "<<q.total()<<std::endl;
		if(q.total()==0){
			for(auto& track:confirmed_tracks_){
				track->MarkMissed();
			}
		}else{
			MeaisAsso = analyze_tracks(q);//分析哪些measure没有关联
			const Matrices& association_matrices = generate_hypothesis(q);//生成假设矩阵

			Eigen::MatrixXd beta = joint_probability(association_matrices, selected_detections);//JPDAF

			std::cout<<"########## BETA ##########\n"<<beta<<std::endl;

			int i=0;
			for(const auto& track:confirmed_tracks_){
		    	std::cout<<"MeaisAsso "<<MeaisAsso[i]<<std::endl;
				if(MeaisAsso[i]){
					std::vector<Eigen::VectorXd> Zv;
					for(auto det : selected_detections){
						Zv.push_back(det.position);
					}
					track->Update(Zv, beta.col(i), beta(beta.rows() - 1, i), time);//TODO change the imm_ukf
				}else{
					track->MarkMissed();
				}
				++i;
			}
		}*/

		std::vector<int> final_select;
		pruning(selected_detections,final_select,prun_tracks);//TODO 剪枝

		for(auto& tr:tracks_){
			Eigen::VectorXd x = tr->GetState();
			Eigen::Vector2f p = tr->GetMeasure();
			int id = tr->GetId();
			Eigen::VectorXd save(11);//id, fx, fy, angle, mx, my, yaw, l, w, h, z
			Box tempb = tr->GetBox();
			save<<id,x(0),x(1),x(2), p(0), p(1), tempb.yaw,tempb.length,tempb.width,tempb.height,tempb.z;
			//std::cout<<"track->Age() "<<tr->Age()<<std::endl;
			if(tr->Age()<3)
				result.push_back(save);
		}


		//################################## 普通IMM-UKF ######################################
		if(unconfirmed_tracks_.size()>0){
			Detection seconda_sso_detections;

			//std::cout<<not_associated_.size()<<std::endl;
			if(not_associated_.size()==0){
				for(const auto& track:unconfirmed_tracks_){
					track->MarkMissed();
				}
			}

			for(auto det:not_associated_){
				seconda_sso_detections.push_back(det);
			}

			not_associated_.clear();
			not_associated_.swap(not_associated_);
			//std::cout<<"######## not asso clear ###############"<<not_associated_.size()<<std::endl;

			for(const auto& track:unconfirmed_tracks_){
				//std::cout<<"########################## unconfirm preditcion ########################"<<std::endl;
				track->Prediction(time);
			}

			const uint& UconfirmTrackSize = unconfirmed_tracks_.size();//这里指的上一时刻那些没有匹配或是没有形成track的目标
			const uint& detSize = seconda_sso_detections.size(); //当前时刻没有匹配的目标

			cv::Mat assigmentsBin = cv::Mat::zeros(cv::Size(detSize, UconfirmTrackSize), CV_32SC1);
			cv::Mat costMat = cv::Mat(cv::Size(detSize, UconfirmTrackSize), CV_32FC1);//cosmatrix (cols rows)
			cv::Mat IoucostMat = cv::Mat(cv::Size(detSize, UconfirmTrackSize), CV_32FC1);//cosmatrix (cols rows)

			std::vector<int> assignments;
			std::vector<float> costs(detSize * UconfirmTrackSize);
			std::vector<float> ioucosts(detSize * UconfirmTrackSize);

			for(uint i = 0; i < UconfirmTrackSize; ++i){
				for(uint j = 0; j < detSize; ++j){
					costs.at(i + j * UconfirmTrackSize ) = euclideanDist(seconda_sso_detections[j].position,
		  				unconfirmed_tracks_[i]->GetZ());
					//ioucosts.at(i + j * prevDetSize ) = RectIou(not_associated_.at(j).rotbox, prev_detections_.at(i).rotbox);
					costMat.at<float>(i, j) = costs.at(i + j * UconfirmTrackSize );
				}
			}
			//std::cout<<"##########  second costMat #####\n"<<costMat<<std::endl;

			AssignmentProblemSolver APS;//匈牙利算法
			APS.Solve(costs, UconfirmTrackSize, detSize, assignments, AssignmentProblemSolver::optimal);

			const uint& assSize = assignments.size();//这个的大小应该是检测结果的大小，里边对应的是目标的编号
			for(uint i = 0; i < assSize; ++i){
				if( assignments[i] != -1 && costMat.at<float>(i, assignments[i]) < param_.pdist_thresh){
					assigmentsBin.at<int>(i, assignments[i]) = 1;
				}
			}
			const uint& rows = assigmentsBin.rows;
			const uint& cols = assigmentsBin.cols;

			std::vector<bool> choosen(detSize,false);
			std::vector<bool> trackst(rows,false);
			std::vector<bool> detst(cols,false);
			for(uint i = 0; i < rows; ++i){
				for(uint j = 0; j < cols; ++j){
					if(assigmentsBin.at<int>(i, j)){
						unconfirmed_tracks_[i]->Update(seconda_sso_detections[j].position);//TODO change the imm_ukf
						unconfirmed_tracks_[i]->UpdateBox(seconda_sso_detections[j]);
						unconfirmed_tracks_[i]->UpdateMeasure(seconda_sso_detections[j].position(0),seconda_sso_detections[j].position(1));
						trackst[i] = true;
						detst[j] = true;
					}
				}
			}

			for(uint j = 0; j < cols; ++j){
				if(!detst[j]){
					not_associated_.push_back(seconda_sso_detections[j]);
				}
			}
			for(uint i=0 ; i<rows; ++i){
				if(!trackst[i]){
					unconfirmed_tracks_[i]->MarkMissed();
				}
			}

		}

		//std::cout<<"######## not asso ###############"<<not_associated_.size()<<std::endl;
		/*for(auto& track:confirmed_tracks_){
			tracks_.push_back(track);
			Eigen::VectorXd x = track->GetState();
			int id = track->GetId();
			Eigen::VectorXd save(3);
			save<<id,x(0),x(1);
			if(track->Age()==0)
				result.push_back(save);
		}*/
		for(auto& track:unconfirmed_tracks_){
			tracks_.push_back(track);
			Eigen::VectorXd x = track->GetState();
			Eigen::Vector2f p = track->GetMeasure();
			int id = track->GetId();
			Eigen::VectorXd save(11);
			Box tempb = track->GetBox();
			save<<id,x(0),x(1), x(2), p(0), p(1), tempb.yaw,tempb.length,tempb.width, tempb.height,tempb.z;
			if(track->Age()<=1)
				result.push_back(save);
		}

		//std::cout<<"tracks "<<tracks_.size()<<std::endl;
		confirmed_tracks_.clear();
		confirmed_tracks_.swap(confirmed_tracks_);
		unconfirmed_tracks_.clear();
		unconfirmed_tracks_.swap(unconfirmed_tracks_);

		delete_tracks();
		manage_tracks(time);
		//std::cout<<"tracks2 "<<tracks_.size()<<std::endl;

		pretime = time;
	}
}

void Tracker::associate(Detection& _selected_detections, cv::Mat& _q, 
			const Detection& _detections)
{	

	//std::cout<<"#################### TRACKER ASSOCISTATE #########################"<<std::endl;
	//Extracting the measurements inside the validation gate for all the tracks
  	//Create a q matrix with a width = clutter + number of tracks
  	_q = cv::Mat_<int>(cv::Size(confirmed_tracks_.size() + 1, _detections.size()), int(0));
  	uint validationIdx = 0;
  	not_associated_.clear();
  	uint j = 0;

  	for(const auto& detection : _detections){
  		uint i = 1;
    	bool found = false;

    	cv::Mat det_cv(cv::Size(2, 1), CV_64FC1);
    	det_cv.at<double>(0) = detection.position(0);
    	det_cv.at<double>(1) = detection.position(1);

    	for(auto& track : confirmed_tracks_){
      		const Eigen::VectorXd& tr = track->GetZ();//TODO GET STATE VECTOR
			
      		cv::Mat tr_cv(cv::Size(2, 1), CV_64FC1);
      		tr_cv.at<double>(0) = tr(0);
      		tr_cv.at<double>(1) = tr(1);
      		const float& Sdt = track->S().determinant();

			//std::cout<<"########## TRACKER Z  ##########\n "<<track->S()<<
					//" \n"<<"########## TRACKER  P ##########\n "<< track->GetZ()<<"\n"<<track->S().determinant()<<std::endl;


			const Eigen::MatrixXd& Sin = track->S().inverse();//TODO GET MEASURE COVARIANCE
      		cv::Mat S_cv;
      		cv::eigen2cv(Sin, S_cv);
      		const double& mah = cv::Mahalanobis(tr_cv, det_cv, S_cv);
      		const float& eucl = euclideanDist(detection.position, tr);

			//std::cout<<"########## TRACKER Mahalanobis euclideanDist ##########\n "<<mah<<" "<<eucl<<" "<<(param_.pi * param_.pg_sigma * std::sqrt(fabs(Sdt)))<<std::endl;
			if(std::isnan(mah) || std::isnan(eucl) ||std::isnan((param_.pi * param_.pg_sigma * std::sqrt(fabs(Sdt)))))
				std::abort();
			//mah <= (param_.pi * param_.pg_sigma * std::sqrt(fabs(Sdt))) &&
			if((eucl <= 1 && mah<chi2in975[2]) || eucl< 1.2){
    	  		_q.at<int>(validationIdx, 0) = 1;
    	  		_q.at<int>(validationIdx, i) = 1;
    	  		found = true;
      		}
      			++i;
    	}

    	if(found){
      		_selected_detections.push_back(detection);//有关联的检测
      		validationIdx++;//实际的关联矩阵的列数，也就是实际detection的数量
    	}else{
      		not_associated_.push_back(detection);
    	}
   		++j;
  	}
	//std::cout<<"######## assosiate not asso ###############"<<not_associated_.size()<<std::endl;

  	_q = _q(cv::Rect(0, 0, confirmed_tracks_.size() + 1, validationIdx));
  ////////////////////////////////////////////////////////////////////////////
}
