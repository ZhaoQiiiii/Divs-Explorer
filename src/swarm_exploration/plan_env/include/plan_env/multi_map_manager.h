#ifndef _MULTI_MAP_MANAGER_H
#define _MULTI_MAP_MANAGER_H

#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>

#include <plan_env/ChunkStamps.h>
#include <plan_env/ChunkData.h>

#include <memory>
#include <random>
#include <vector>
#include <unordered_map>

using std::shared_ptr;
using std::vector;
using std::unordered_map;

namespace fast_planner {
class SDFMap;
class MapROS;

struct MapChunk {
  // Map Chunk: the elementary exchange unit between robots

  uint32_t idx_;
  vector<uint32_t> voxel_adrs_;
  vector<uint8_t> voxel_occ_;

  bool need_query_;
  bool empty_;
};

struct ChunksData {
  // Chunks Data: the container that saves multi map chunks

  vector<MapChunk> chunks_;
  vector<int> idx_list_; 
};

struct ChunksBox {
  Eigen::Vector3d min_;
  Eigen::Vector3d max_;
  bool valid_;
};

class MultiMapManager {
public:
  MultiMapManager();
  ~MultiMapManager();
  void setMap(SDFMap* map);
  void init();

  // Main API
  void updateMapChunk(const vector<uint32_t>& adrs);
  void getChunkBoxes(vector<Eigen::Vector3d>& mins, vector<Eigen::Vector3d>& maxs, bool reset = true);

private:
  void sendChunks(const int& chunk_drone_id, const int& to_drone_id, const vector<int>& idx_list);
  void getOccOfChunk(const vector<uint32_t>& adrs, vector<uint8_t>& occs);
  void insertChunkToMap(const MapChunk& chunk, const int& chunk_drone_id);
  void adrToIndex(const uint32_t& adr, Eigen::Vector3i& idx);

  void stampTimerCallback(const ros::TimerEvent& e);
  void chunkTimerCallback(const ros::TimerEvent& e);
  void stampMsgCallback(const plan_env::ChunkStampsConstPtr& msg);
  void chunkCallback(const plan_env::ChunkDataConstPtr& msg);

  // Operations on the chunk idx list
  void findMissedChunkIds(const vector<int>& self_idx_list, const vector<int>& other_idx_list, vector<int>& miss_ids);
  bool findIntersect(const int& min1, const int& max1, const int& min2, const int max2, int& minr, int& maxr);
  void mergeChunkIds(const vector<int>& list1, const vector<int>& list2, vector<int>& output);

  // Data
  int drone_id_, map_num_;
  int vis_drone_id_;  // ONLY use for ground node!
  int chunk_size_;

  SDFMap* map_;
  ros::NodeHandle nh_;
  ros::Publisher stamp_pub_, chunk_pub_, marker_pub_;
  ros::Subscriber stamp_sub_, chunk_sub_;
  ros::Timer stamp_timer_, chunk_timer_;

  vector<Eigen::Vector3i> tmp_ids_;
  Eigen::Vector3d drone_pos_;

  // Main multi-map data
  vector<ChunksData> multi_map_chunks_;               // Container for chunks data of all drones 
  vector<uint32_t> adr_buffer_;                       // Buffer for chunks data of this map
  vector<vector<plan_env::ChunkData>> chunk_buffer_;  // Buffer for chunks data of external map (all drones)

  vector<unordered_map<int, char>> buffer_map_;       // Hash map to avoid repeated insertion of chunk msg
  vector<ChunksBox> chunk_boxes_;                     // Bounding box of map chunks of swarm
  vector<double> last_chunk_stamp_time_;              // Time Table for recording stamp that send chunks data

  friend SDFMap;
  friend MapROS;
};

}  // namespace fast_planner

#endif