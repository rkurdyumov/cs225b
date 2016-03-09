/*
 * Copyright 2010 SRI International
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <sstream>
#include <fstream>
#include <stdexcept>
#include <queue>
#include <set>
#include <list>
#include <iterator>

#include <math.h>
#include <assert.h>

#include "Mapper.h"

namespace karto
{

  // enable this for verbose debug information
  //#define KARTO_DEBUG

  #define MAX_VARIANCE            500.0
  #define DISTANCE_PENALTY_GAIN   0.2
  #define ANGLE_PENALTY_GAIN      0.2

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Manages the scan data for a device
   */
  class ScanManager
  {
  public:
    /**
     * Default constructor
     */
    ScanManager(kt_int32u runningBufferMaximumSize, kt_double runningBufferMaximumDistance)
      : m_pLastScan(NULL)
      , m_RunningBufferMaximumSize(runningBufferMaximumSize)
      , m_RunningBufferMaximumDistance(runningBufferMaximumDistance)
    {
    }

    /**
     * Destructor
     */
    virtual ~ScanManager()
    {
      Clear();
    }

  public:
    /**
     * Adds scan to vector of processed scans tagging scan with given unique id
     * @param pScan
     */
    inline void AddScan(LocalizedRangeScan* pScan, kt_int32s uniqueId)
    {
      // assign state id to scan
      pScan->SetStateId(static_cast<kt_int32u>(m_Scans.size()));

      // assign unique id to scan
      pScan->SetUniqueId(uniqueId);

      // add it to scan buffer
      m_Scans.push_back(pScan);
    }

    /**
     * Gets last scan
     * @param deviceId
     * @return last localized range scan
     */
    inline LocalizedRangeScan* GetLastScan()
    {
      return m_pLastScan;
    }

    /**
     * Sets the last scan
     * @param pScan
     */
    inline void SetLastScan(LocalizedRangeScan* pScan)
    {
      m_pLastScan = pScan;
    }

    /**
     * Gets scans
     * @return scans
     */
    inline LocalizedRangeScanVector& GetScans()
    {
      return m_Scans;
    }

    /**
     * Gets running scans
     * @return running scans
     */
    inline LocalizedRangeScanVector& GetRunningScans()
    {
      return m_RunningScans;
    }

    /**
     * Adds scan to vector of running scans
     * @param pScan
     */
    void AddRunningScan(LocalizedRangeScan* pScan)
    {
      m_RunningScans.push_back(pScan);

      // vector has at least one element (first line of this function), so this is valid
      Pose2 frontScanPose = m_RunningScans.front()->GetSensorPose();
      Pose2 backScanPose = m_RunningScans.back()->GetSensorPose();

      // cap vector size and remove all scans from front of vector that are too far from end of vector
      kt_double squaredDistance = frontScanPose.GetPosition().SquaredDistance(backScanPose.GetPosition());
      while (m_RunningScans.size() > m_RunningBufferMaximumSize || squaredDistance > math::Square(m_RunningBufferMaximumDistance) - KT_TOLERANCE)
      {
        // remove front of running scans
        m_RunningScans.erase(m_RunningScans.begin());

        // recompute stats of running scans
        frontScanPose = m_RunningScans.front()->GetSensorPose();
        backScanPose = m_RunningScans.back()->GetSensorPose();
        squaredDistance = frontScanPose.GetPosition().SquaredDistance(backScanPose.GetPosition());
      }
    }

    /**
     * Deletes data of this buffered device
     */
    void Clear()
    {
      m_Scans.clear();
      m_RunningScans.clear();
    }

  private:
    LocalizedRangeScanVector m_Scans;
    LocalizedRangeScanVector m_RunningScans;
    LocalizedRangeScan* m_pLastScan;

    kt_int32u m_RunningBufferMaximumSize;
    kt_double m_RunningBufferMaximumDistance;
  }; // ScanManager

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  void MapperSensorManager::RegisterSensor(const Name& rSensorName)
  {
    if(GetScanManager(rSensorName) == NULL)
    {
      m_ScanManagers[rSensorName] = new ScanManager(m_RunningBufferMaximumSize, m_RunningBufferMaximumDistance);
    }
  }


  /**
   * Gets scan from given device with given ID
   * @param rSensorName
   * @param scanNum
   * @return localized range scan
   */
  LocalizedRangeScan* MapperSensorManager::GetScan(const Name& rSensorName, kt_int32s scanIndex)
  {
    ScanManager* pScanManager = GetScanManager(rSensorName);
    if(pScanManager != NULL)
    {
      return pScanManager->GetScans().at(scanIndex);
    }

    assert(false);
    return NULL;
  }

  /**
   * Gets last scan of given device
   * @param pLaserRangeFinder
   * @return last localized range scan of device
   */
  inline LocalizedRangeScan* MapperSensorManager::GetLastScan(const Name& rSensorName)
  {
    RegisterSensor(rSensorName);

    return GetScanManager(rSensorName)->GetLastScan();
  }

  /**
   * Sets the last scan of device of given scan
   * @param pScan
   */
  inline void MapperSensorManager::SetLastScan(LocalizedRangeScan* pScan)
  {
    GetScanManager(pScan)->SetLastScan(pScan);
  }

  /**
   * Adds scan to scan vector of device that recorded scan
   * @param pScan
   */
  void MapperSensorManager::AddScan(LocalizedRangeScan* pScan)
  {
    GetScanManager(pScan)->AddScan(pScan, m_NextScanId);
    m_Scans.push_back(pScan);
    m_NextScanId++;
  }

  /**
   * Adds scan to running scans of device that recorded scan
   * @param pScan
   */
  inline void MapperSensorManager::AddRunningScan(LocalizedRangeScan* pScan)
  {
    GetScanManager(pScan)->AddRunningScan(pScan);
  }

  /**
   * Gets scans of device
   * @param rSensorName
   * @return scans of device
   */
  inline LocalizedRangeScanVector& MapperSensorManager::GetScans(const Name& rSensorName)
  {
    return GetScanManager(rSensorName)->GetScans();
  }

  /**
   * Gets running scans of device
   * @param rSensorName
   * @return running scans of device
   */
  inline LocalizedRangeScanVector& MapperSensorManager::GetRunningScans(const Name& rSensorName)
  {
    return GetScanManager(rSensorName)->GetRunningScans();
  }

  /**
   * Gets all scans of all devices
   * @return all scans of all devices
   */
  LocalizedRangeScanVector MapperSensorManager::GetAllScans()
  {
    LocalizedRangeScanVector scans;

    forEach(ScanManagerMap, &m_ScanManagers)
    {
      LocalizedRangeScanVector& rScans = iter->second->GetScans();

      scans.insert(scans.end(), rScans.begin(), rScans.end());
    }

    return scans;
  }

  /**
   * Deletes all scan managers of all devices
   */
  void MapperSensorManager::Clear()
  {
//    SensorManager::Clear();

    forEach(ScanManagerMap, &m_ScanManagers)
    {
      delete iter->second;
    }

    m_ScanManagers.clear();
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  ScanMatcher::~ScanMatcher()
  {
    delete m_pCorrelationGrid;
    delete m_pSearchSpaceProbs;
    delete m_pGridLookup;
  }

  ScanMatcher* ScanMatcher::Create(Mapper* pMapper, kt_double searchSize, kt_double resolution, kt_double smearDeviation, kt_double rangeThreshold)
  {
    // invalid parameters
    if (resolution <= 0)
    {
      return NULL;
    }
    if (searchSize <= 0)
    {
      return NULL;
    }
    if (smearDeviation < 0)
    {
      return NULL;
    }
    if (rangeThreshold <= 0)
    {
      return NULL;
    }

    assert(math::DoubleEqual(math::Round(searchSize / resolution), (searchSize / resolution)));

    // calculate search space in grid coordinates
    kt_int32u searchSpaceSideSize = static_cast<kt_int32u>(math::Round(searchSize / resolution) + 1);

    // compute requisite size of correlation grid (pad grid so that scan points can't fall off the grid
    // if a scan is on the border of the search space)
    kt_int32u pointReadingMargin = static_cast<kt_int32u>(ceil(rangeThreshold / resolution));

    kt_int32s gridSize = searchSpaceSideSize + 2 * pointReadingMargin;

    // create correlation grid
    assert(gridSize % 2 == 1);
    CorrelationGrid* pCorrelationGrid = CorrelationGrid::CreateGrid(gridSize, gridSize, resolution, smearDeviation);

    // create search space probabilities
    Grid<kt_double>* pSearchSpaceProbs = Grid<kt_double>::CreateGrid(searchSpaceSideSize, searchSpaceSideSize, resolution);

    ScanMatcher* pScanMatcher = new ScanMatcher(pMapper);
    pScanMatcher->m_pCorrelationGrid = pCorrelationGrid;
    pScanMatcher->m_pSearchSpaceProbs = pSearchSpaceProbs;
    pScanMatcher->m_pGridLookup = new GridIndexLookup<kt_int8u>(pCorrelationGrid);

    return pScanMatcher;
  }

  /**
   * Match given scan against set of scans
   * @param pScan scan being scan-matched
   * @param rBaseScans set of scans whose points will mark cells in grid as being occupied
   * @param rMean output parameter of mean (best pose) of match
   * @param rCovariance output parameter of covariance of match
   * @param doPenalize whether to penalize matches further from the search center
   * @param doRefineMatch whether to do finer-grained matching if coarse match is good (default is true)
   * @return strength of response
   */
  kt_double ScanMatcher::MatchScan(LocalizedRangeScan* pScan, const LocalizedRangeScanVector& rBaseScans, Pose2& rMean, Matrix3& rCovariance,
                                   kt_bool doPenalize, kt_bool doRefineMatch)
  {
    ///////////////////////////////////////
    // set scan pose to be center of grid

    // 1. get scan position
    Pose2 scanPose = pScan->GetSensorPose();

    // scan has no readings; cannot do scan matching
    // best guess of pose is based off of adjusted odometer reading
    if (pScan->GetNumberOfRangeReadings() == 0)
    {
      rMean = scanPose;

      // maximum covariance
      rCovariance(0, 0) = MAX_VARIANCE; // XX
      rCovariance(1, 1) = MAX_VARIANCE; // YY
      rCovariance(2, 2) = 4 * math::Square(m_pMapper->m_pCoarseAngleResolution->GetValue()); // TH*TH

      return 0.0;
    }

    // 2. get size of grid
    Rectangle2<kt_int32s> roi = m_pCorrelationGrid->GetROI();

    // 3. compute offset (in meters - lower left corner)
    Vector2<kt_double> offset;
    offset.SetX(scanPose.GetX() - (0.5 * (roi.GetWidth() - 1) * m_pCorrelationGrid->GetResolution()));
    offset.SetY(scanPose.GetY() - (0.5 * (roi.GetHeight() - 1) * m_pCorrelationGrid->GetResolution()));

    // 4. set offset
    m_pCorrelationGrid->GetCoordinateConverter()->SetOffset(offset);

    ///////////////////////////////////////

    // set up correlation grid
    AddScans(rBaseScans, scanPose.GetPosition());

    // compute how far to search in each direction
    Vector2<kt_double> searchDimensions(m_pSearchSpaceProbs->GetWidth(), m_pSearchSpaceProbs->GetHeight());
    Vector2<kt_double> coarseSearchOffset(0.5 * (searchDimensions.GetX() - 1) * m_pCorrelationGrid->GetResolution(), 0.5 * (searchDimensions.GetY() - 1) * m_pCorrelationGrid->GetResolution());

    // a coarse search only checks half the cells in each dimension
    Vector2<kt_double> coarseSearchResolution(2 * m_pCorrelationGrid->GetResolution(), 2 * m_pCorrelationGrid->GetResolution());

    // actual scan-matching
    kt_double bestResponse = CorrelateScan(pScan, scanPose,	coarseSearchOffset, coarseSearchResolution, m_pMapper->m_pCoarseSearchAngleOffset->GetValue(), m_pMapper->m_pCoarseAngleResolution->GetValue(), doPenalize, rMean, rCovariance, false);

    if (m_pMapper->m_pUseResponseExpansion->GetValue() == true)
    {
      if (math::DoubleEqual(bestResponse, 0.0))
      {
#ifdef KARTO_DEBUG
        std::cout << "Mapper Info: Expanding response search space!" << std::endl;
#endif
        // try and increase search angle offset with 20 degrees and do another match
        kt_double newSearchAngleOffset = m_pMapper->m_pCoarseSearchAngleOffset->GetValue();
        for (kt_int32u i = 0; i < 3; i++)
        {
          newSearchAngleOffset += math::DegreesToRadians(20);

          bestResponse = CorrelateScan(pScan, scanPose,	coarseSearchOffset, coarseSearchResolution, newSearchAngleOffset, m_pMapper->m_pCoarseAngleResolution->GetValue(), doPenalize, rMean, rCovariance, false);

          if (math::DoubleEqual(bestResponse, 0.0) == false)
          {
            break;
          }
        }

#ifdef KARTO_DEBUG
        if (math::DoubleEqual(bestResponse, 0.0))
        {
          std::cout << "Mapper Warning: Unable to calculate response!" << std::endl;
        }
#endif
      }
    }

    if (doRefineMatch)
    {
      Vector2<kt_double> fineSearchOffset(coarseSearchResolution * 0.5);
      Vector2<kt_double> fineSearchResolution(m_pCorrelationGrid->GetResolution(), m_pCorrelationGrid->GetResolution());
      bestResponse = CorrelateScan(pScan, rMean, fineSearchOffset, fineSearchResolution, 0.5 * m_pMapper->m_pCoarseAngleResolution->GetValue(), m_pMapper->m_pFineSearchAngleOffset->GetValue(), doPenalize, rMean, rCovariance, true);
    }

#ifdef KARTO_DEBUG
    std::cout << "  BEST POSE = " << rMean << " BEST RESPONSE = " << bestResponse << ",  VARIANCE = " << rCovariance(0, 0) << ", " << rCovariance(1, 1) << std::endl;
#endif

    assert(math::InRange(rMean.GetHeading(), -KT_PI, KT_PI));

    return bestResponse;
  }

  /**
   * Finds the best pose for the scan centering the search in the correlation grid
   * at the given pose and search in the space by the vector and angular offsets
   * in increments of the given resolutions
   * @param rScan scan to match against correlation grid
   * @param rSearchCenter the center of the search space
   * @param rSearchSpaceOffset searches poses in the area offset by this vector around search center
   * @param rSearchSpaceResolution how fine a granularity to search in the search space
   * @param searchAngleOffset searches poses in the angles offset by this angle around search center
   * @param searchAngleResolution how fine a granularity to search in the angular search space
   * @param doPenalize whether to penalize matches further from the search center
   * @param rMean output parameter of mean (best pose) of match
   * @param rCovariance output parameter of covariance of match
   * @param doingFineMatch whether to do a finer search after coarse search
   * @return strength of response
   */
  kt_double ScanMatcher::CorrelateScan(LocalizedRangeScan* pScan, const Pose2& rSearchCenter, const Vector2<kt_double>& rSearchSpaceOffset, const Vector2<kt_double>& rSearchSpaceResolution,
                                       kt_double searchAngleOffset, kt_double searchAngleResolution,	kt_bool doPenalize, Pose2& rMean, Matrix3& rCovariance, kt_bool doingFineMatch)
  {
    assert(searchAngleResolution != 0.0);

    // setup lookup arrays
    m_pGridLookup->ComputeOffsets(pScan, rSearchCenter.GetHeading(), searchAngleOffset, searchAngleResolution);

    // only initialize probability grid if computing positional covariance (during coarse match)
    if (!doingFineMatch)
    {
      m_pSearchSpaceProbs->Clear();

      // position search grid - finds lower left corner of search grid
      Vector2<kt_double> offset(rSearchCenter.GetPosition() - rSearchSpaceOffset);
      m_pSearchSpaceProbs->GetCoordinateConverter()->SetOffset(offset);
    }

    // calculate position arrays

    std::vector<kt_double> xPoses;
    kt_int32u nX = static_cast<kt_int32u>(math::Round(rSearchSpaceOffset.GetX() * 2.0 / rSearchSpaceResolution.GetX()) + 1);
    kt_double startX = -rSearchSpaceOffset.GetX();
    for (kt_int32u xIndex = 0; xIndex < nX; xIndex++)
    {
      xPoses.push_back(startX + xIndex * rSearchSpaceResolution.GetX());
    }
    assert(math::DoubleEqual(xPoses.back(), -startX));

    std::vector<kt_double> yPoses;
    kt_int32u nY = static_cast<kt_int32u>(math::Round(rSearchSpaceOffset.GetY() * 2.0 / rSearchSpaceResolution.GetY()) + 1);
    kt_double startY = -rSearchSpaceOffset.GetY();
    for (kt_int32u yIndex = 0; yIndex < nY; yIndex++)
    {
      yPoses.push_back(startY + yIndex * rSearchSpaceResolution.GetY());
    }
    assert(math::DoubleEqual(yPoses.back(), -startY));

    // calculate pose response array size
    kt_int32u nAngles = static_cast<kt_int32u>(math::Round(searchAngleOffset * 2.0 / searchAngleResolution) + 1);

    kt_int32u poseResponseSize = static_cast<kt_int32u>(xPoses.size() * yPoses.size() * nAngles);

    // allocate array
    std::pair<kt_double, Pose2>* pPoseResponse = new std::pair<kt_double, Pose2>[poseResponseSize];

    Vector2<kt_int32s> startGridPoint = m_pCorrelationGrid->WorldToGrid(Vector2<kt_double>(rSearchCenter.GetX() + startX, rSearchCenter.GetY() + startY));

    kt_int32u poseResponseCounter = 0;
    forEachAs(std::vector<kt_double>, &yPoses, yIter)
    {
      kt_double y = *yIter;
      kt_double newPositionY = rSearchCenter.GetY() + y;
      kt_double squareY = math::Square(y);

      forEachAs(std::vector<kt_double>, &xPoses, xIter)
      {
        kt_double x = *xIter;
        kt_double newPositionX = rSearchCenter.GetX() + x;
        kt_double squareX = math::Square(x);

        Vector2<kt_int32s> gridPoint = m_pCorrelationGrid->WorldToGrid(Vector2<kt_double>(newPositionX, newPositionY));
        kt_int32s gridIndex = m_pCorrelationGrid->GridIndex(gridPoint);
        assert(gridIndex >= 0);

        kt_double angle = 0.0;
        kt_double startAngle = rSearchCenter.GetHeading() - searchAngleOffset;
        for (kt_int32u angleIndex = 0; angleIndex < nAngles; angleIndex++)
        {
          angle = startAngle + angleIndex * searchAngleResolution;

          kt_double response = GetResponse(angleIndex, gridIndex);
          if (doPenalize && (math::DoubleEqual(response, 0.0) == false))
          {
            // simple model (approximate Gaussian) to take odometry into account

            kt_double squaredDistance = squareX + squareY;
            kt_double distancePenalty = 1.0 - (DISTANCE_PENALTY_GAIN * squaredDistance / m_pMapper->m_pDistanceVariancePenalty->GetValue());
            distancePenalty = math::Maximum(distancePenalty, m_pMapper->m_pMinimumDistancePenalty->GetValue());

            kt_double squaredAngleDistance = math::Square(angle - rSearchCenter.GetHeading());
            kt_double anglePenalty = 1.0 - (ANGLE_PENALTY_GAIN * squaredAngleDistance / m_pMapper->m_pAngleVariancePenalty->GetValue());
            anglePenalty = math::Maximum(anglePenalty, m_pMapper->m_pMinimumAnglePenalty->GetValue());

            response *= (distancePenalty * anglePenalty);
          }

          // store response and pose
          pPoseResponse[poseResponseCounter] = std::pair<kt_double, Pose2>(response, Pose2(newPositionX, newPositionY, math::NormalizeAngle(angle)));
          poseResponseCounter++;
        }

        assert(math::DoubleEqual(angle, rSearchCenter.GetHeading() + searchAngleOffset));
      }
    }

    assert(poseResponseSize == poseResponseCounter);

    // find value of best response (in [0; 1])
    kt_double bestResponse = -1;
    for (kt_int32u i = 0; i < poseResponseSize; i++)
    {
      bestResponse = math::Maximum(bestResponse, pPoseResponse[i].first);

      // will compute positional covariance, save best relative probability for each cell
      if (!doingFineMatch)
      {
        const Pose2& rPose = pPoseResponse[i].second;
        Vector2<kt_int32s> grid = m_pSearchSpaceProbs->WorldToGrid(rPose.GetPosition());

        kt_double* ptr = (kt_double*)m_pSearchSpaceProbs->GetDataPointer(grid);
        if (ptr == NULL)
        {
          throw std::runtime_error("Mapper FATAL ERROR - Index out of range in probability search!");
        }

        *ptr = math::Maximum(pPoseResponse[i].first, *ptr);
      }
    }

    // average all poses with same highest response
    Vector2<kt_double> averagePosition;
    kt_double thetaX = 0.0;
    kt_double thetaY = 0.0;
    kt_int32s averagePoseCount = 0;
    for (kt_int32u i = 0; i < poseResponseSize; i++)
    {
      if (math::DoubleEqual(pPoseResponse[i].first, bestResponse))
      {
        averagePosition += pPoseResponse[i].second.GetPosition();

        kt_double heading = pPoseResponse[i].second.GetHeading();
        thetaX += cos(heading);
        thetaY += sin(heading);

        averagePoseCount++;
      }
    }

    Pose2 averagePose;
    if (averagePoseCount > 0)
    {
      averagePosition /= averagePoseCount;

      thetaX /= averagePoseCount;
      thetaY /= averagePoseCount;

      averagePose = Pose2(averagePosition, atan2(thetaY, thetaX));
    }
    else
    {
      throw std::runtime_error("Mapper FATAL ERROR - Unable to find best position");
    }

    // delete pose response array
    delete [] pPoseResponse;

#ifdef KARTO_DEBUG
    std::cout << "bestPose: " << averagePose << std::endl;
    std::cout << "bestResponse: " << bestResponse << std::endl;
#endif

    if (!doingFineMatch)
    {
      ComputePositionalCovariance(averagePose, bestResponse, rSearchCenter, rSearchSpaceOffset, rSearchSpaceResolution, searchAngleResolution, rCovariance);
    }
    else
    {
      ComputeAngularCovariance(averagePose, bestResponse, rSearchCenter, searchAngleOffset, searchAngleResolution, rCovariance);
    }

    rMean = averagePose;

#ifdef KARTO_DEBUG
    std::cout << "bestPose: " << averagePose << std::endl;
#endif

    if (bestResponse > 1.0)
    {
      bestResponse = 1.0;
    }

    assert(math::InRange(bestResponse, 0.0, 1.0));
    assert(math::InRange(rMean.GetHeading(), -KT_PI, KT_PI));

    return bestResponse;
  }

  /**
   * Computes the positional covariance of the best pose
   * @param rBestPose
   * @param bestResponse
   * @param rSearchCenter
   * @param rSearchSpaceOffset
   * @param rSearchSpaceResolution
   * @param searchAngleResolution
   * @param rCovariance
   */
  void ScanMatcher::ComputePositionalCovariance(const Pose2& rBestPose, kt_double bestResponse, const Pose2& rSearchCenter,
                                                const Vector2<kt_double>& rSearchSpaceOffset, const Vector2<kt_double>& rSearchSpaceResolution,
                                                kt_double searchAngleResolution, Matrix3& rCovariance)
  {
    // reset covariance to identity matrix
    rCovariance.SetToIdentity();

    // if best response is vary small return max variance
    if (bestResponse < KT_TOLERANCE)
    {
      rCovariance(0, 0) = MAX_VARIANCE; // XX
      rCovariance(1, 1) = MAX_VARIANCE; // YY
      rCovariance(2, 2) = 4 * math::Square(searchAngleResolution); // TH*TH

      return;
    }

    kt_double accumulatedVarianceXX = 0;
    kt_double accumulatedVarianceXY = 0;
    kt_double accumulatedVarianceYY = 0;
    kt_double norm = 0;

    kt_double dx = rBestPose.GetX() - rSearchCenter.GetX();
    kt_double dy = rBestPose.GetY() - rSearchCenter.GetY();

    kt_double offsetX = rSearchSpaceOffset.GetX();
    kt_double offsetY = rSearchSpaceOffset.GetY();

    kt_int32u nX = static_cast<kt_int32u>(math::Round(offsetX * 2.0 / rSearchSpaceResolution.GetX()) + 1);
    kt_double startX = -offsetX;
    assert(math::DoubleEqual(startX + (nX - 1) * rSearchSpaceResolution.GetX(), -startX));

    kt_int32u nY = static_cast<kt_int32u>(math::Round(offsetY * 2.0 / rSearchSpaceResolution.GetY()) + 1);
    kt_double startY = -offsetY;
    assert(math::DoubleEqual(startY + (nY - 1) * rSearchSpaceResolution.GetY(), -startY));

    for (kt_int32u yIndex = 0; yIndex < nY; yIndex++)
    {
      kt_double y = startY + yIndex * rSearchSpaceResolution.GetY();

      for (kt_int32u xIndex = 0; xIndex < nX; xIndex++)
      {
        kt_double x = startX + xIndex * rSearchSpaceResolution.GetX();

        Vector2<kt_int32s> gridPoint = m_pSearchSpaceProbs->WorldToGrid(Vector2<kt_double>(rSearchCenter.GetX() + x, rSearchCenter.GetY() + y));
        kt_double response = *(m_pSearchSpaceProbs->GetDataPointer(gridPoint));

        // response is not a low response
        if (response >= (bestResponse - 0.1))
        {
          norm += response;
          accumulatedVarianceXX += (math::Square(x - dx) * response);
          accumulatedVarianceXY += ((x - dx) * (y - dy) * response);
          accumulatedVarianceYY += (math::Square(y - dy) * response);
        }
      }
    }

    if (norm > KT_TOLERANCE)
    {
      kt_double varianceXX = accumulatedVarianceXX / norm;
      kt_double varianceXY = accumulatedVarianceXY / norm;
      kt_double varianceYY = accumulatedVarianceYY / norm;
      kt_double varianceTHTH = 4 * math::Square(searchAngleResolution);

      // lower-bound variances so that they are not too small;
      // ensures that links are not too tight
      kt_double minVarianceXX = 0.1 * math::Square(rSearchSpaceResolution.GetX());
      kt_double minVarianceYY = 0.1 * math::Square(rSearchSpaceResolution.GetY());
      varianceXX = math::Maximum(varianceXX, minVarianceXX);
      varianceYY = math::Maximum(varianceYY, minVarianceYY);

      // increase variance for poorer responses
      kt_double multiplier = 1.0 / bestResponse;
      rCovariance(0, 0) = varianceXX * multiplier;
      rCovariance(0, 1) = varianceXY * multiplier;
      rCovariance(1, 0) = varianceXY * multiplier;
      rCovariance(1, 1) = varianceYY * multiplier;
      rCovariance(2, 2) = varianceTHTH; // this value will be set in ComputeAngularCovariance
    }

    // if values are 0, set to MAX_VARIANCE
    // values might be 0 if points are too sparse and thus don't hit other points
    if (math::DoubleEqual(rCovariance(0, 0), 0.0))
    {
      rCovariance(0, 0) = MAX_VARIANCE;
    }

    if (math::DoubleEqual(rCovariance(1, 1), 0.0))
    {
      rCovariance(1, 1) = MAX_VARIANCE;
    }
  }

  /**
   * Computes the angular covariance of the best pose
   * @param rBestPose
   * @param bestResponse
   * @param rSearchCenter
   * @param rSearchAngleOffset
   * @param searchAngleResolution
   * @param rCovariance
   */
  void ScanMatcher::ComputeAngularCovariance(const Pose2& rBestPose, kt_double bestResponse, const Pose2& rSearchCenter,
                                             kt_double searchAngleOffset, kt_double searchAngleResolution, Matrix3& rCovariance)
  {
    // NOTE: do not reset covariance matrix

    // normalize angle difference
    kt_double bestAngle = math::NormalizeAngleDifference(rBestPose.GetHeading(), rSearchCenter.GetHeading());

    Vector2<kt_int32s> gridPoint = m_pCorrelationGrid->WorldToGrid(rBestPose.GetPosition());
    kt_int32s gridIndex = m_pCorrelationGrid->GridIndex(gridPoint);

    kt_int32u nAngles = static_cast<kt_int32u>(math::Round(searchAngleOffset * 2 / searchAngleResolution) + 1);

    kt_double angle = 0.0;
    kt_double startAngle = rSearchCenter.GetHeading() - searchAngleOffset;

    kt_double norm = 0.0;
    kt_double accumulatedVarianceThTh = 0.0;
    for (kt_int32u angleIndex = 0; angleIndex < nAngles; angleIndex++)
    {
      angle = startAngle + angleIndex * searchAngleResolution;
      kt_double response = GetResponse(angleIndex, gridIndex);

      // response is not a low response
      if (response >= (bestResponse - 0.1))
      {
        norm += response;
        accumulatedVarianceThTh += (math::Square(angle - bestAngle) * response);
      }
    }
    assert(math::DoubleEqual(angle, rSearchCenter.GetHeading() + searchAngleOffset));

    if (norm > KT_TOLERANCE)
    {
      if (accumulatedVarianceThTh < KT_TOLERANCE)
      {
        accumulatedVarianceThTh = math::Square(searchAngleResolution);
      }

      accumulatedVarianceThTh /= norm;
    }
    else
    {
      accumulatedVarianceThTh = 1000 * math::Square(searchAngleResolution);
    }

    rCovariance(2, 2) = accumulatedVarianceThTh;
  }

  /**
   * Marks cells where scans' points hit as being occupied
   * @param rScans scans whose points will mark cells in grid as being occupied
   * @param viewPoint do not add points that belong to scans "opposite" the view point
   */
  void ScanMatcher::AddScans(const LocalizedRangeScanVector& rScans, Vector2<kt_double> viewPoint)
  {
    m_pCorrelationGrid->Clear();

    // add all scans to grid
    const_forEach(LocalizedRangeScanVector, &rScans)
    {
      AddScan(*iter, viewPoint);
    }
  }

  /**
   * Marks cells where scans' points hit as being occupied.  Can smear points as they are added.
   * @param pScan scan whose points will mark cells in grid as being occupied
   * @param viewPoint do not add points that belong to scans "opposite" the view point
   * @param doSmear whether the points will be smeared
   */
  void ScanMatcher::AddScan(LocalizedRangeScan* pScan, const Vector2<kt_double>& rViewPoint, kt_bool doSmear)
  {
    PointVectorDouble validPoints = FindValidPoints(pScan, rViewPoint);

    // put in all valid points
    const_forEach(PointVectorDouble, &validPoints)
    {
      Vector2<kt_int32s> gridPoint = m_pCorrelationGrid->WorldToGrid(*iter);
      if (!math::IsUpTo(gridPoint.GetX(), m_pCorrelationGrid->GetROI().GetWidth()) || !math::IsUpTo(gridPoint.GetY(), m_pCorrelationGrid->GetROI().GetHeight()))
      {
        // point not in grid
        continue;
      }

      int gridIndex = m_pCorrelationGrid->GridIndex(gridPoint);

      // set grid cell as occupied
      if (m_pCorrelationGrid->GetDataPointer()[gridIndex] == GridStates_Occupied)
      {
        // value already set
        continue;
      }

      m_pCorrelationGrid->GetDataPointer()[gridIndex] = GridStates_Occupied;

      // smear grid
      if (doSmear == true)
      {
        m_pCorrelationGrid->SmearPoint(gridPoint);
      }
    }
  }

  /**
   * Compute which points in a scan are on the same side as the given viewpoint
   * @param pScan
   * @param rViewPoint
   * @return points on the same side
   */
  PointVectorDouble ScanMatcher::FindValidPoints(LocalizedRangeScan* pScan, const Vector2<kt_double>& rViewPoint) const
  {
    const PointVectorDouble& rPointReadings = pScan->GetPointReadings();

    // points must be at least 10 cm away when making comparisons of inside/outside of viewpoint
    const kt_double minSquareDistance = math::Square(0.1); // in m^2

    // this iterator lags from the main iterator adding points only when the points are on
    // the same side as the viewpoint
    PointVectorDouble::const_iterator trailingPointIter = rPointReadings.begin();
    PointVectorDouble validPoints;

    Vector2<kt_double> firstPoint;
    kt_bool firstTime = true;
    const_forEach(PointVectorDouble, &rPointReadings)
    {
      Vector2<kt_double> currentPoint = *iter;

      if (firstTime)
      {
        firstPoint = currentPoint;
        firstTime = false;
      }

      Vector2<kt_double> delta = firstPoint - currentPoint;
      if (delta.SquaredLength() > minSquareDistance)
      {
        // This compute the Determinant (viewPoint FirstPoint, viewPoint currentPoint)
        // Which computes the direction of rotation, if the rotation is counterclock
        // wise then we are looking at data we should keep. If it's negative rotation
        // we should not included in in the matching
        // have enough distance, check viewpoint
        double a = rViewPoint.GetY() - firstPoint.GetY();
        double b = firstPoint.GetX() - rViewPoint.GetX();
        double c = firstPoint.GetY() * rViewPoint.GetX() - firstPoint.GetX() * rViewPoint.GetY();
        double ss = currentPoint.GetX() * a + currentPoint.GetY() * b + c;

        // reset beginning point
        firstPoint = currentPoint;

        if (ss < 0.0)	// wrong side, skip and keep going
        {
          trailingPointIter = iter;
        }
        else
        {
          for (; trailingPointIter != iter; trailingPointIter++)
          {
            validPoints.push_back(*trailingPointIter);
          }
        }
      }
    }

    return validPoints;
  }

  /**
   * Get response at given position for given rotation (only look up valid points)
   * @param angleIndex
   * @param gridPositionIndex
   * @return response
   */
  kt_double ScanMatcher::GetResponse(kt_int32u angleIndex, kt_int32s gridPositionIndex) const
  {
    kt_double response = 0.0;

    // add up value for each point
    kt_int8u* pByte = m_pCorrelationGrid->GetDataPointer() + gridPositionIndex;

    const LookupArray* pOffsets = m_pGridLookup->GetLookupArray(angleIndex);
    assert(pOffsets != NULL);

    // get number of points in offset list
    kt_int32u nPoints = pOffsets->GetSize();
    if (nPoints == 0)
    {
      return response;
    }

    // calculate response
    kt_int32s* pAngleIndexPointer = pOffsets->GetArrayPointer();
    for (kt_int32u i = 0; i < nPoints; i++)
    {
      // ignore points that fall off the grid
      kt_int32s pointGridIndex = gridPositionIndex + pAngleIndexPointer[i];
      if (!math::IsUpTo(pointGridIndex, m_pCorrelationGrid->GetDataSize()))
      {
        continue;
      }

      // uses index offsets to efficiently find location of point in the grid
      response += pByte[pAngleIndexPointer[i]];
    }

    // normalize response
    response /= (nPoints * GridStates_Occupied);
    assert(fabs(response) <= 1.0);

    return response;
  }


  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  template<typename T>
  class BreadthFirstTraversal : public GraphTraversal<T>
  {
  public:
    /**
     * Constructs a breadth-first traverser for the given graph
     */
    BreadthFirstTraversal(Graph<T>* pGraph)
    : GraphTraversal<T>(pGraph)
    {
    }

    /**
     * Destructor
     */
    virtual ~BreadthFirstTraversal()
    {
    }

  public:
    /**
     * Traverse the graph starting with the given vertex; applies the visitor to visited nodes
     * @param pStartVertex
     * @param pVisitor
     * @return visited vertices
     */
    virtual std::vector<T*> Traverse(Vertex<T>* pStartVertex, Visitor<T>* pVisitor)
    {
      std::queue<Vertex<T>*> toVisit;
      std::set<Vertex<T>*> seenVertices;
      std::vector<Vertex<T>*> validVertices;

      toVisit.push(pStartVertex);
      seenVertices.insert(pStartVertex);

      do
      {
        Vertex<T>* pNext = toVisit.front();
        toVisit.pop();

        if (pVisitor->Visit(pNext))
        {
          // vertex is valid, explore neighbors
          validVertices.push_back(pNext);

          std::vector<Vertex<T>*> adjacentVertices = pNext->GetAdjacentVertices();
          forEach(typename std::vector<Vertex<T>*>, &adjacentVertices)
          {
            Vertex<T>* pAdjacent = *iter;

            // adjacent vertex has not yet been seen, add to queue for processing
            if (seenVertices.find(pAdjacent) == seenVertices.end())
            {
              toVisit.push(pAdjacent);
              seenVertices.insert(pAdjacent);
            }
          }
        }
      } while (toVisit.empty() == false);

      std::vector<T*> objects;
      forEach(typename std::vector<Vertex<T>*>, &validVertices)
      {
        objects.push_back((*iter)->GetObject());
      }

      return objects;
    }

  }; // class BreadthFirstTraversal

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  class NearScanVisitor : public Visitor<LocalizedRangeScan>
  {
  public:
    NearScanVisitor(LocalizedRangeScan* pScan, kt_double maxDistance, kt_bool useScanBarycenter)
      : m_MaxDistanceSquared(math::Square(maxDistance))
      , m_UseScanBarycenter(useScanBarycenter)
    {
      m_CenterPose = pScan->GetReferencePose(m_UseScanBarycenter);
    }

    virtual kt_bool Visit(Vertex<LocalizedRangeScan>* pVertex)
    {
      LocalizedRangeScan* pScan = pVertex->GetObject();

      Pose2 pose = pScan->GetReferencePose(m_UseScanBarycenter);

      kt_double squaredDistance = pose.GetPosition().SquaredDistance(m_CenterPose.GetPosition());
      return (squaredDistance <= m_MaxDistanceSquared - KT_TOLERANCE);
    }

  protected:
    Pose2 m_CenterPose;
    kt_double m_MaxDistanceSquared;
    kt_bool m_UseScanBarycenter;

  }; // NearScanVisitor

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  MapperGraph::MapperGraph(Mapper* pMapper, kt_double rangeThreshold)
    : m_pMapper(pMapper)
  {
    m_pLoopScanMatcher = ScanMatcher::Create(pMapper, m_pMapper->m_pLoopSearchSpaceDimension->GetValue(), m_pMapper->m_pLoopSearchSpaceResolution->GetValue(), m_pMapper->m_pLoopSearchSpaceSmearDeviation->GetValue(), rangeThreshold);
    assert(m_pLoopScanMatcher);

    m_pTraversal = new BreadthFirstTraversal<LocalizedRangeScan>(this);
  }

  MapperGraph::~MapperGraph()
  {
    delete m_pLoopScanMatcher;
    m_pLoopScanMatcher = NULL;

    delete m_pTraversal;
    m_pTraversal = NULL;
  }

  void MapperGraph::AddVertex(LocalizedRangeScan* pScan)
  {
    assert(pScan);

    if(pScan != NULL)
    {
      Vertex<LocalizedRangeScan>* pVertex = new Vertex<LocalizedRangeScan>(pScan);
      Graph<LocalizedRangeScan>::AddVertex(pScan->GetSensorName(), pVertex);
      if (m_pMapper->m_pScanOptimizer != NULL)
      {
        m_pMapper->m_pScanOptimizer->AddNode(pVertex);
      }
    }
  }

  void MapperGraph::AddEdges(LocalizedRangeScan* pScan, const Matrix3& rCovariance)
  {
    MapperSensorManager* pSensorManager = m_pMapper->m_pMapperSensorManager;

    const Name& rSensorName = pScan->GetSensorName();

    // link to previous scan
    kt_int32s previousScanNum = pScan->GetStateId() - 1;
    if (pSensorManager->GetLastScan(rSensorName) != NULL)
    {
      assert(previousScanNum >= 0);
      LinkScans(pSensorManager->GetScan(rSensorName, previousScanNum), pScan, pScan->GetSensorPose(), rCovariance);
    }

    Pose2Vector means;
    std::vector<Matrix3> covariances;

    // first scan (link to first scan of other robots)
    if (pSensorManager->GetLastScan(rSensorName) == NULL)
    {
      assert(pSensorManager->GetScans(rSensorName).size() == 1);

      std::vector<Name> deviceNames = pSensorManager->GetSensorNames();
      forEach(std::vector<Name>, &deviceNames)
      {
        const Name& rCandidateSensorName = *iter;

        // skip if candidate device is the same or other device has no scans
        if ((rCandidateSensorName == rSensorName) || (pSensorManager->GetScans(rCandidateSensorName).empty()))
        {
          continue;
        }

        Pose2 bestPose;
        Matrix3 covariance;
        kt_double response = m_pMapper->m_pSequentialScanMatcher->MatchScan(pScan, pSensorManager->GetScans(rCandidateSensorName), bestPose, covariance);
        LinkScans(pScan, pSensorManager->GetScan(rCandidateSensorName, 0), bestPose, covariance);

        // only add to means and covariances if response was high "enough"
        if (response > m_pMapper->m_pLinkMatchMinimumResponseFine->GetValue())
        {
          means.push_back(bestPose);
          covariances.push_back(covariance);
        }
      }
    }
    else
    {
      // link to running scans
      Pose2 scanPose = pScan->GetSensorPose();
      means.push_back(scanPose);
      covariances.push_back(rCovariance);
      LinkChainToScan(pSensorManager->GetRunningScans(rSensorName), pScan, scanPose, rCovariance);
    }

    // link to other near chains (chains that include new scan are invalid)
    LinkNearChains(pScan, means, covariances);

    if (!means.empty())
    {
      pScan->SetSensorPose(ComputeWeightedMean(means, covariances));
    }
  }

  kt_bool MapperGraph::TryCloseLoop(LocalizedRangeScan* pScan, const Name& rSensorName)
  {
    kt_bool loopClosed = false;

    kt_int32u scanIndex = 0;

    LocalizedRangeScanVector candidateChain = FindPossibleLoopClosure(pScan, rSensorName, scanIndex);

    while (!candidateChain.empty())
    {
      Pose2 bestPose;
      Matrix3 covariance;
      kt_double coarseResponse = m_pLoopScanMatcher->MatchScan(pScan, candidateChain, bestPose, covariance, false, false);

      std::stringstream stream;
      stream << "COARSE RESPONSE: " << coarseResponse << " (&gt; " << m_pMapper->m_pLoopMatchMinimumResponseCoarse->GetValue() << ")" << std::endl;
      stream << "            var: " << covariance(0, 0) << ",  " << covariance(1, 1) << " (&lt; " << m_pMapper->m_pLoopMatchMaximumVarianceCoarse->GetValue() << ")";
      m_pMapper->FireLoopClosureCheck(stream.str());

      if ((coarseResponse > m_pMapper->m_pLoopMatchMinimumResponseCoarse->GetValue()) &&
          (covariance(0, 0) < m_pMapper->m_pLoopMatchMaximumVarianceCoarse->GetValue()) &&
          (covariance(1, 1) < m_pMapper->m_pLoopMatchMaximumVarianceCoarse->GetValue()))
      {
        // save for reversion
        Pose2 oldPose = pScan->GetSensorPose();

        pScan->SetSensorPose(bestPose);
        kt_double fineResponse = m_pMapper->m_pSequentialScanMatcher->MatchScan(pScan, candidateChain, bestPose, covariance, false);

        std::stringstream stream1;
        stream1 << "FINE RESPONSE: " << fineResponse << " (>" << m_pMapper->m_pLoopMatchMinimumResponseFine->GetValue() << ")" << std::endl;
        m_pMapper->FireLoopClosureCheck(stream1.str());

        if (fineResponse < m_pMapper->m_pLoopMatchMinimumResponseFine->GetValue())
        {
          // failed verification test, revert
          pScan->SetSensorPose(oldPose);

          m_pMapper->FireLoopClosureCheck("REJECTED!");
        }
        else
        {
          m_pMapper->FireBeginLoopClosure("Closing loop...");

          pScan->SetSensorPose(bestPose);
          LinkChainToScan(candidateChain, pScan, bestPose, covariance);
          CorrectPoses();

          m_pMapper->FireEndLoopClosure("Loop closed!");

          loopClosed = true;
        }
      }

      candidateChain = FindPossibleLoopClosure(pScan, rSensorName, scanIndex);
    }

    return loopClosed;
  }

  LocalizedRangeScan* MapperGraph::GetClosestScanToPose(const LocalizedRangeScanVector& rScans, const Pose2& rPose) const
  {
    LocalizedRangeScan* pClosestScan = NULL;
    kt_double bestSquaredDistance = DBL_MAX;

    const_forEach(LocalizedRangeScanVector, &rScans)
    {
      Pose2 scanPose = (*iter)->GetReferencePose(m_pMapper->m_pUseScanBarycenter->GetValue());

      kt_double squaredDistance = rPose.GetPosition().SquaredDistance(scanPose.GetPosition());
      if (squaredDistance < bestSquaredDistance)
      {
        bestSquaredDistance = squaredDistance;
        pClosestScan = *iter;
      }
    }

    return pClosestScan;
  }

  Edge<LocalizedRangeScan>* MapperGraph::AddEdge(LocalizedRangeScan* pSourceScan, LocalizedRangeScan* pTargetScan, kt_bool& rIsNewEdge)
  {
    // check that vertex exists
    assert(pSourceScan->GetStateId() < (kt_int32s)m_Vertices[pSourceScan->GetSensorName()].size());
    assert(pTargetScan->GetStateId() < (kt_int32s)m_Vertices[pTargetScan->GetSensorName()].size());

    Vertex<LocalizedRangeScan>* v1 = m_Vertices[pSourceScan->GetSensorName()][pSourceScan->GetStateId()];
    Vertex<LocalizedRangeScan>* v2 = m_Vertices[pTargetScan->GetSensorName()][pTargetScan->GetStateId()];

    // see if edge already exists
    const_forEach(std::vector<Edge<LocalizedRangeScan>*>, &(v1->GetEdges()))
    {
      Edge<LocalizedRangeScan>* pEdge = *iter;

      if (pEdge->GetTarget() == v2)
      {
        rIsNewEdge = false;
        return pEdge;
      }
    }

    Edge<LocalizedRangeScan>* pEdge = new Edge<LocalizedRangeScan>(v1, v2);
    Graph<LocalizedRangeScan>::AddEdge(pEdge);
    rIsNewEdge = true;
    return pEdge;
  }

  void MapperGraph::LinkScans(LocalizedRangeScan* pFromScan, LocalizedRangeScan* pToScan, const Pose2& rMean, const Matrix3& rCovariance)
  {
    kt_bool isNewEdge = true;
    Edge<LocalizedRangeScan>* pEdge = AddEdge(pFromScan, pToScan, isNewEdge);

    // only attach link information if the edge is new
    if (isNewEdge == true)
    {
      pEdge->SetLabel(new LinkInfo(pFromScan->GetSensorPose(), rMean, rCovariance));
      if (m_pMapper->m_pScanOptimizer != NULL)
      {
        m_pMapper->m_pScanOptimizer->AddConstraint(pEdge);
      }
    }
  }

  void MapperGraph::LinkNearChains(LocalizedRangeScan* pScan, Pose2Vector& rMeans, std::vector<Matrix3>& rCovariances)
  {
    const std::vector<LocalizedRangeScanVector> nearChains = FindNearChains(pScan);
    const_forEach(std::vector<LocalizedRangeScanVector>, &nearChains)
    {
      if (iter->size() < m_pMapper->m_pLoopMatchMinimumChainSize->GetValue())
      {
        continue;
      }

      Pose2 mean;
      Matrix3 covariance;
      // match scan against "near" chain
      kt_double response = m_pMapper->m_pSequentialScanMatcher->MatchScan(pScan, *iter, mean, covariance, false);
      if (response > m_pMapper->m_pLinkMatchMinimumResponseFine->GetValue() - KT_TOLERANCE)
      {
        rMeans.push_back(mean);
        rCovariances.push_back(covariance);
        LinkChainToScan(*iter, pScan, mean, covariance);
      }
    }
  }

  void MapperGraph::LinkChainToScan(const LocalizedRangeScanVector& rChain, LocalizedRangeScan* pScan, const Pose2& rMean, const Matrix3& rCovariance)
  {
    Pose2 pose = pScan->GetReferencePose(m_pMapper->m_pUseScanBarycenter->GetValue());

    LocalizedRangeScan* pClosestScan = GetClosestScanToPose(rChain, pose);
    assert(pClosestScan != NULL);

    Pose2 closestScanPose = pClosestScan->GetReferencePose(m_pMapper->m_pUseScanBarycenter->GetValue());

    kt_double squaredDistance = pose.GetPosition().SquaredDistance(closestScanPose.GetPosition());
    if (squaredDistance < math::Square(m_pMapper->m_pLinkScanMaximumDistance->GetValue()) + KT_TOLERANCE)
    {
      LinkScans(pClosestScan, pScan, rMean, rCovariance);
    }
  }

  std::vector<LocalizedRangeScanVector> MapperGraph::FindNearChains(LocalizedRangeScan* pScan)
  {
    std::vector<LocalizedRangeScanVector> nearChains;

    Pose2 scanPose = pScan->GetReferencePose(m_pMapper->m_pUseScanBarycenter->GetValue());

    // to keep track of which scans have been added to a chain
    LocalizedRangeScanVector processed;

    const LocalizedRangeScanVector nearLinkedScans = FindNearLinkedScans(pScan, m_pMapper->m_pLinkScanMaximumDistance->GetValue());
    const_forEach(LocalizedRangeScanVector, &nearLinkedScans)
    {
      LocalizedRangeScan* pNearScan = *iter;

      if (pNearScan == pScan)
      {
        continue;
      }

      // scan has already been processed, skip
      if (find(processed.begin(), processed.end(), pNearScan) != processed.end())
      {
        continue;
      }

      processed.push_back(pNearScan);

      // build up chain
      kt_bool isValidChain = true;
      std::list<LocalizedRangeScan*> chain;

      // add scans before current scan being processed
      for (kt_int32s candidateScanNum = pNearScan->GetStateId() - 1; candidateScanNum >= 0; candidateScanNum--)
      {
        LocalizedRangeScan* pCandidateScan = m_pMapper->m_pMapperSensorManager->GetScan(pNearScan->GetSensorName(), candidateScanNum);

        // chain is invalid--contains scan being added
        if (pCandidateScan == pScan)
        {
          isValidChain = false;
        }

        Pose2 candidatePose = pCandidateScan->GetReferencePose(m_pMapper->m_pUseScanBarycenter->GetValue());
        kt_double squaredDistance = scanPose.GetPosition().SquaredDistance(candidatePose.GetPosition());

        if (squaredDistance < math::Square(m_pMapper->m_pLinkScanMaximumDistance->GetValue()) + KT_TOLERANCE)
        {
          chain.push_front(pCandidateScan);
          processed.push_back(pCandidateScan);
        }
        else
        {
          break;
        }
      }

      chain.push_back(pNearScan);

      // add scans after current scan being processed
      kt_int32u end = static_cast<kt_int32u>(m_pMapper->m_pMapperSensorManager->GetScans(pNearScan->GetSensorName()).size());
      for (kt_int32u candidateScanNum = pNearScan->GetStateId() + 1; candidateScanNum < end; candidateScanNum++)
      {
        LocalizedRangeScan* pCandidateScan = m_pMapper->m_pMapperSensorManager->GetScan(pNearScan->GetSensorName(), candidateScanNum);

        if (pCandidateScan == pScan)
        {
          isValidChain = false;
        }

        Pose2 candidatePose = pCandidateScan->GetReferencePose(m_pMapper->m_pUseScanBarycenter->GetValue());;
        kt_double squaredDistance = scanPose.GetPosition().SquaredDistance(candidatePose.GetPosition());

        if (squaredDistance < math::Square(m_pMapper->m_pLinkScanMaximumDistance->GetValue()) + KT_TOLERANCE)
        {
          chain.push_back(pCandidateScan);
          processed.push_back(pCandidateScan);
        }
        else
        {
          break;
        }
      }

      if (isValidChain)
      {
        // change list to vector
        LocalizedRangeScanVector tempChain;
        std::copy(chain.begin(), chain.end(), std::inserter(tempChain, tempChain.begin()));
        // add chain to collection
        nearChains.push_back(tempChain);
      }
    }

    return nearChains;
  }

  LocalizedRangeScanVector MapperGraph::FindNearLinkedScans(LocalizedRangeScan* pScan, kt_double maxDistance)
  {
    NearScanVisitor* pVisitor = new NearScanVisitor(pScan, maxDistance, m_pMapper->m_pUseScanBarycenter->GetValue());
    LocalizedRangeScanVector nearLinkedScans = m_pTraversal->Traverse(GetVertex(pScan), pVisitor);
    delete pVisitor;

    return nearLinkedScans;
  }

  Pose2 MapperGraph::ComputeWeightedMean(const Pose2Vector& rMeans, const std::vector<Matrix3>& rCovariances) const
  {
    assert(rMeans.size() == rCovariances.size());

    // compute sum of inverses and create inverse list
    std::vector<Matrix3> inverses;
    inverses.reserve(rCovariances.size());

    Matrix3 sumOfInverses;
    const_forEach(std::vector<Matrix3>, &rCovariances)
    {
      Matrix3 inverse = iter->Inverse();
      inverses.push_back(inverse);

      sumOfInverses += inverse;
    }
    Matrix3 inverseOfSumOfInverses = sumOfInverses.Inverse();

    // compute weighted mean
    Pose2 accumulatedPose;
    kt_double thetaX = 0.0;
    kt_double thetaY = 0.0;

    Pose2Vector::const_iterator meansIter = rMeans.begin();
    const_forEach(std::vector<Matrix3>, &inverses)
    {
      Pose2 pose = *meansIter;
      kt_double angle = pose.GetHeading();
      thetaX += cos(angle);
      thetaY += sin(angle);

      Matrix3 weight = inverseOfSumOfInverses * (*iter);
      accumulatedPose += weight * pose;

      meansIter++;
    }

    thetaX /= rMeans.size();
    thetaY /= rMeans.size();
    accumulatedPose.SetHeading(atan2(thetaY, thetaX));

    return accumulatedPose;
  }

  LocalizedRangeScanVector MapperGraph::FindPossibleLoopClosure(LocalizedRangeScan* pScan, const Name& rSensorName, kt_int32u& rStartNum)
  {
    LocalizedRangeScanVector chain; // return value

    Pose2 pose = pScan->GetReferencePose(m_pMapper->m_pUseScanBarycenter->GetValue());

    // possible loop closure chain should not include close scans that have a
    // path of links to the scan of interest
    const LocalizedRangeScanVector nearLinkedScans = FindNearLinkedScans(pScan, m_pMapper->m_pLoopSearchMaximumDistance->GetValue());

    kt_int32u nScans = static_cast<kt_int32u>(m_pMapper->m_pMapperSensorManager->GetScans(rSensorName).size());
    for (; rStartNum < nScans; rStartNum++)
    {
      LocalizedRangeScan* pCandidateScan = m_pMapper->m_pMapperSensorManager->GetScan(rSensorName, rStartNum);

      Pose2 candidateScanPose = pCandidateScan->GetReferencePose(m_pMapper->m_pUseScanBarycenter->GetValue());

      kt_double squaredDistance = candidateScanPose.GetPosition().SquaredDistance(pose.GetPosition());
      if (squaredDistance < math::Square(m_pMapper->m_pLoopSearchMaximumDistance->GetValue()) + KT_TOLERANCE)
      {
        // a linked scan cannot be in the chain
        if (find(nearLinkedScans.begin(), nearLinkedScans.end(), pCandidateScan) != nearLinkedScans.end())
        {
          chain.clear();
        }
        else
        {
          chain.push_back(pCandidateScan);
        }
      }
      else
      {
        // return chain if it is long "enough"
        if (chain.size() >= m_pMapper->m_pLoopMatchMinimumChainSize->GetValue())
        {
          return chain;
        }
        else
        {
          chain.clear();
        }
      }
    }

    return chain;
  }

  void MapperGraph::CorrectPoses()
  {
    // optimize scans!
    ScanSolver* pSolver = m_pMapper->m_pScanOptimizer;
    if (pSolver != NULL)
    {
      pSolver->Compute();

      const_forEach(ScanSolver::IdPoseVector, &pSolver->GetCorrections())
      {
        m_pMapper->m_pMapperSensorManager->GetScan(iter->first)->SetSensorPose(iter->second);
      }

      pSolver->Clear();
    }
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Default constructor
   */
  Mapper::Mapper()
    : Module("Mapper")
    , m_Initialized(false)
    , m_pSequentialScanMatcher(NULL)
    , m_pMapperSensorManager(NULL)
    , m_pGraph(NULL)
    , m_pScanOptimizer(NULL)
  {
    InitializeParameters();
  }

  /**
   * Default constructor
   */
  Mapper::Mapper(const std::string& rName)
    : Module(rName)
    , m_Initialized(false)
    , m_pSequentialScanMatcher(NULL)
    , m_pMapperSensorManager(NULL)
    , m_pGraph(NULL)
    , m_pScanOptimizer(NULL)
  {
    InitializeParameters();
  }

  /**
   * Destructor
   */
  Mapper::~Mapper()
  {
    Reset();

    delete m_pMapperSensorManager;
  }

  void Mapper::InitializeParameters()
  {
    /**
     * When set to true, the mapper will use a scan matching algorithm. In most real-world situations
     * this should be set to true so that the mapper algorithm can correct for noise and errors in
     * odometry and scan data. In some simulator environments where the simulated scan and odometry
     * data are very accurate, the scan matching algorithm can produce worse results. In those cases
     * set this to false to improve results.
     * Default value is true.
     */
    m_pUseScanMatching = new Parameter<kt_bool>("UseScanMatching", true, GetParameterManager());

    /**
     * Default value is true.
     */
    m_pUseScanBarycenter = new Parameter<kt_bool>("UseScanBarycenter", true, GetParameterManager());

    /**
     * Sets the minimum travel between scans.  If a new scan's position is more than minimumTravelDistance
     * from the previous scan, the mapper will use the data from the new scan. Otherwise, it will discard the
     * new scan if it also does not meet the minimum change in heading requirement.
     * For performance reasons, generally it is a good idea to only process scans if the robot
     * has moved a reasonable amount.
     * Default value is 0.2 (meters).
     */
    m_pMinimumTravelDistance = new Parameter<kt_double>("MinimumTravelDistance", 0.2, GetParameterManager());

    /**
     * Sets the minimum heading change between scans. If a new scan's heading is more than minimumTravelHeading
     * from the previous scan, the mapper will use the data from the new scan.  Otherwise, it will discard the
     * new scan if it also does not meet the minimum travel distance requirement.
     * For performance reasons, generally it is a good idea to only process scans if the robot
     * has moved a reasonable amount.
     * Default value is 10 degrees.
     */
    m_pMinimumTravelHeading = new Parameter<kt_double>("MinimumTravelHeading", math::DegreesToRadians(10), GetParameterManager());

    /**
     * Scan buffer size is the length of the scan chain stored for scan matching.
     * "scanBufferSize" should be set to approximately "scanBufferMaximumScanDistance" / "minimumTravelDistance".
     * The idea is to get an area approximately 20 meters long for scan matching.
     * For example, if we add scans every minimumTravelDistance == 0.3 meters, then "scanBufferSize"
     * should be 20 / 0.3 = 67.)
     * Default value is 67.
     */
    m_pScanBufferSize = new Parameter<kt_int32u>("ScanBufferSize", 70, GetParameterManager());

    /**
     * Scan buffer maximum scan distance is the maximum distance between the first and last scans
     * in the scan chain stored for matching.
     * Default value is 20.0.
     */
    m_pScanBufferMaximumScanDistance = new Parameter<kt_double>("ScanBufferMaximumScanDistance", 20.0, GetParameterManager());

    /**
     * Scans are linked only if the correlation response value is greater than this value.
     * Default value is 0.4
     */
    m_pLinkMatchMinimumResponseFine = new Parameter<kt_double>("LinkMatchMinimumResponseFine", 0.8, GetParameterManager());

    /**
     * Maximum distance between linked scans.  Scans that are farther apart will not be linked
     * regardless of the correlation response value.
     * Default value is 6.0 meters.
     */
    m_pLinkScanMaximumDistance = new Parameter<kt_double>("LinkScanMaximumDistance", 10.0, GetParameterManager());

    /**
     * Scans less than this distance from the current position will be considered for a match
     * in loop closure.
     * Default value is 4.0 meters.
     */
    m_pLoopSearchMaximumDistance = new Parameter<kt_double>("LoopSearchMaximumDistance", 4.0, GetParameterManager());

    /**
     * When the loop closure detection finds a candidate it must be part of a large
     * set of linked scans. If the chain of scans is less than this value we do not attempt
     * to close the loop.
     * Default value is 10.
     */
    m_pLoopMatchMinimumChainSize = new Parameter<kt_int32u>("LoopMatchMinimumChainSize", 10, GetParameterManager());

    /**
     * The co-variance values for a possible loop closure have to be less than this value
     * to consider a viable solution. This applies to the coarse search.
     * Default value is 0.16.
     */
    m_pLoopMatchMaximumVarianceCoarse = new Parameter<kt_double>("LoopMatchMaximumVarianceCoarse", math::Square(0.4), GetParameterManager());

    /**
     * If response is larger then this, then initiate loop closure search at the coarse resolution.
     * Default value is 0.7.
     */
    m_pLoopMatchMinimumResponseCoarse = new Parameter<kt_double>("LoopMatchMinimumResponseCoarse", 0.8, GetParameterManager());

    /**
     * If response is larger then this, then initiate loop closure search at the fine resolution.
     * Default value is 0.7.
     */
    m_pLoopMatchMinimumResponseFine = new Parameter<kt_double>("LoopMatchMinimumResponseFine", 0.8, GetParameterManager());

    //////////////////////////////////////////////////////////////////////////////
    //    CorrelationParameters correlationParameters;

    /**
     * The size of the search grid used by the matcher.
     * Default value is 0.3 meters which tells the matcher to use a 30cm x 30cm grid.
     */
    m_pCorrelationSearchSpaceDimension = new Parameter<kt_double>("CorrelationSearchSpaceDimension", 0.3, GetParameterManager());

    /**
     * The resolution (size of a grid cell) of the correlation grid.
     * Default value is 0.01 meters.
     */
    m_pCorrelationSearchSpaceResolution = new Parameter<kt_double>("CorrelationSearchSpaceResolution", 0.01, GetParameterManager());

    /**
     * The point readings are smeared by this value in X and Y to create a smoother response.
     * Default value is 0.03 meters.
     */
    m_pCorrelationSearchSpaceSmearDeviation = new Parameter<kt_double>("CorrelationSearchSpaceSmearDeviation", 0.03, GetParameterManager());


    //////////////////////////////////////////////////////////////////////////////
    //    CorrelationParameters loopCorrelationParameters;

    /**
     * The size of the search grid used by the matcher.
     * Default value is 0.3 meters which tells the matcher to use a 30cm x 30cm grid.
     */
    m_pLoopSearchSpaceDimension = new Parameter<kt_double>("LoopSearchSpaceDimension", 8.0, GetParameterManager());

    /**
     * The resolution (size of a grid cell) of the correlation grid.
     * Default value is 0.01 meters.
     */
    m_pLoopSearchSpaceResolution = new Parameter<kt_double>("LoopSearchSpaceResolution", 0.05, GetParameterManager());

    /**
     * The point readings are smeared by this value in X and Y to create a smoother response.
     * Default value is 0.03 meters.
     */
    m_pLoopSearchSpaceSmearDeviation = new Parameter<kt_double>("LoopSearchSpaceSmearDeviation", 0.03, GetParameterManager());

    //////////////////////////////////////////////////////////////////////////////
    // ScanMatcherParameters;

    // Variance of penalty for deviating from odometry when scan-matching.
    // The penalty is a multiplier (less than 1.0) is a function of the
    // delta of the scan position being tested and the odometric pose
    m_pDistanceVariancePenalty = new Parameter<kt_double>("DistanceVariancePenalty", math::Square(0.3), GetParameterManager());
    m_pAngleVariancePenalty = new Parameter<kt_double>("AngleVariancePenalty", math::Square(math::DegreesToRadians(20)), GetParameterManager());

    // The range of angles to search during a coarse search and a finer search
    m_pFineSearchAngleOffset = new Parameter<kt_double>("FineSearchAngleOffset", math::DegreesToRadians(0.2), GetParameterManager());
    m_pCoarseSearchAngleOffset = new Parameter<kt_double>("CoarseSearchAngleOffset", math::DegreesToRadians(20), GetParameterManager());

    // Resolution of angles to search during a coarse search
    m_pCoarseAngleResolution = new Parameter<kt_double>("CoarseAngleResolution", math::DegreesToRadians(2), GetParameterManager());

    // Minimum value of the penalty multiplier so scores do not
    // become too small
    m_pMinimumAnglePenalty = new Parameter<kt_double>("MinimumAnglePenalty", 0.9, GetParameterManager());
    m_pMinimumDistancePenalty = new Parameter<kt_double>("MinimumDistancePenalty", 0.5, GetParameterManager());

    // whether to increase the search space if no good matches are initially found
    m_pUseResponseExpansion = new Parameter<kt_bool>("UseResponseExpansion", false, GetParameterManager());
  }

  void Mapper::Initialize(kt_double rangeThreshold)
  {
    if (m_Initialized == false)
    {
      // create sequential scan and loop matcher
      m_pSequentialScanMatcher = ScanMatcher::Create(this, m_pCorrelationSearchSpaceDimension->GetValue(), m_pCorrelationSearchSpaceResolution->GetValue(), m_pCorrelationSearchSpaceSmearDeviation->GetValue(), rangeThreshold);
      assert(m_pSequentialScanMatcher);

      m_pMapperSensorManager = new MapperSensorManager(m_pScanBufferSize->GetValue(), m_pScanBufferMaximumScanDistance->GetValue());

      m_pGraph = new MapperGraph(this, rangeThreshold);

      m_Initialized = true;
    }
  }

  void Mapper::Reset()
  {
    delete m_pSequentialScanMatcher;
    m_pSequentialScanMatcher = NULL;

    delete m_pGraph;
    m_pGraph = NULL;

    delete m_pMapperSensorManager;
    m_pMapperSensorManager = NULL;

    m_Initialized = false;
  }

  kt_bool Mapper::Process(Object* /*pObject*/)
  {
    return true;
  }

  kt_bool Mapper::Process(LocalizedRangeScan* pScan)
  {
    if(pScan != NULL)
    {
      karto::LaserRangeFinder* pLaserRangeFinder = pScan->GetLaserRangeFinder();

      // validate scan
      if (pLaserRangeFinder == NULL || pScan == NULL || pLaserRangeFinder->Validate(pScan) == false)
      {
        return false;
      }

      if (m_Initialized == false)
      {
        // initialize mapper with range threshold from device
        Initialize(pLaserRangeFinder->GetRangeThreshold());
      }

      // get last scan
      LocalizedRangeScan* pLastScan = m_pMapperSensorManager->GetLastScan(pScan->GetSensorName());

      // update scans corrected pose based on last correction
      if (pLastScan != NULL)
      {
        Transform lastTransform(pLastScan->GetOdometricPose(), pLastScan->GetCorrectedPose());
        pScan->SetCorrectedPose(lastTransform.TransformPose(pScan->GetOdometricPose()));
      }

      // test if scan is outside minimum boundary or if heading is larger then minimum heading
      if (!HasMovedEnough(pScan, pLastScan))
      {
        return false;
      }

      Matrix3 covariance;
      covariance.SetToIdentity();

      // correct scan (if not first scan)
      if (m_pUseScanMatching->GetValue() && pLastScan != NULL)
      {
        Pose2 bestPose;
        m_pSequentialScanMatcher->MatchScan(pScan, m_pMapperSensorManager->GetRunningScans(pScan->GetSensorName()), bestPose, covariance);
        pScan->SetSensorPose(bestPose);
      }

      // add scan to buffer and assign id
      m_pMapperSensorManager->AddScan(pScan);

      if (m_pUseScanMatching->GetValue())
      {
        // add to graph
        m_pGraph->AddVertex(pScan);
        m_pGraph->AddEdges(pScan, covariance);

        m_pMapperSensorManager->AddRunningScan(pScan);

        std::vector<Name> deviceNames = m_pMapperSensorManager->GetSensorNames();
        const_forEach(std::vector<Name>, &deviceNames)
        {
          m_pGraph->TryCloseLoop(pScan, *iter);
        }
      }

      m_pMapperSensorManager->SetLastScan(pScan);

      return true;
    }

    return false;
  }

  /**
   * Is the scan sufficiently far from the last scan?
   * @param pScan
   * @param pLastScan
   * @return true if the scans are sufficiently far
   */
  kt_bool Mapper::HasMovedEnough(LocalizedRangeScan* pScan, LocalizedRangeScan* pLastScan) const
  {
    // test if first scan
    if (pLastScan == NULL)
    {
      return true;
    }

    Pose2 lastScannerPose = pLastScan->GetSensorAt(pLastScan->GetOdometricPose());
    Pose2 scannerPose = pScan->GetSensorAt(pScan->GetOdometricPose());

    // test if we have turned enough
    kt_double deltaHeading = math::NormalizeAngle(scannerPose.GetHeading() - lastScannerPose.GetHeading());
    if (fabs(deltaHeading) >= m_pMinimumTravelHeading->GetValue())
    {
      return true;
    }

    // test if we have moved enough
    kt_double squaredTravelDistance = lastScannerPose.GetPosition().SquaredDistance(scannerPose.GetPosition());
    if (squaredTravelDistance >= math::Square(m_pMinimumTravelDistance->GetValue()) - KT_TOLERANCE)
    {
      return true;
    }

    return false;
  }

  /**
   * Gets all the processed scans
   * @return all scans
   */
  const LocalizedRangeScanVector Mapper::GetAllProcessedScans() const
  {
    LocalizedRangeScanVector allScans;

    if (m_pMapperSensorManager != NULL)
    {
      allScans = m_pMapperSensorManager->GetAllScans();
    }

    return allScans;
  }

  /**
   * Adds a listener
   * @param pListener
   */
  void Mapper::AddListener(MapperListener* pListener)
  {
    m_Listeners.push_back(pListener);
  }

  /**
   * Removes a listener
   * @param pListener
   */
  void Mapper::RemoveListener(MapperListener* pListener)
  {
    std::vector<MapperListener*>::iterator iter = std::find(m_Listeners.begin(), m_Listeners.end(), pListener);
    if (iter != m_Listeners.end())
    {
      m_Listeners.erase(iter);
    }
  }

  void Mapper::FireInfo(const std::string& rInfo) const
  {
    const_forEach(std::vector<MapperListener*>, &m_Listeners)
    {
      (*iter)->Info(rInfo);
    }
  }

  void Mapper::FireDebug(const std::string& rInfo) const
  {
    const_forEach(std::vector<MapperListener*>, &m_Listeners)
    {
      MapperDebugListener* pListener = dynamic_cast<MapperDebugListener*>(*iter);

      if (pListener != NULL)
      {
        pListener->Debug(rInfo);
      }
    }
  }

  void Mapper::FireLoopClosureCheck(const std::string& rInfo) const
  {
    const_forEach(std::vector<MapperListener*>, &m_Listeners)
    {
      MapperLoopClosureListener* pListener = dynamic_cast<MapperLoopClosureListener*>(*iter);

      if (pListener != NULL)
      {
        pListener->LoopClosureCheck(rInfo);
      }
    }
  }

  void Mapper::FireBeginLoopClosure(const std::string& rInfo) const
  {
    const_forEach(std::vector<MapperListener*>, &m_Listeners)
    {
      MapperLoopClosureListener* pListener = dynamic_cast<MapperLoopClosureListener*>(*iter);

      if (pListener != NULL)
      {
        pListener->BeginLoopClosure(rInfo);
      }
    }
  }

  void Mapper::FireEndLoopClosure(const std::string& rInfo) const
  {
    const_forEach(std::vector<MapperListener*>, &m_Listeners)
    {
      MapperLoopClosureListener* pListener = dynamic_cast<MapperLoopClosureListener*>(*iter);

      if (pListener != NULL)
      {
        pListener->EndLoopClosure(rInfo);
      }
    }
  }

  void Mapper::SetScanSolver(ScanSolver* pScanOptimizer)
  {
    m_pScanOptimizer = pScanOptimizer;
  }

  MapperGraph* Mapper::GetGraph() const
  {
    return m_pGraph;
  }

  ScanMatcher* Mapper::GetSequentialScanMatcher() const
  {
    return m_pSequentialScanMatcher;
  }

  ScanMatcher* Mapper::GetLoopScanMatcher() const
  {
    return m_pGraph->GetLoopScanMatcher();
  }
}