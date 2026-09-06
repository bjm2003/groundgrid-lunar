#include "groundgrid/PlanningSnapshot.h"

#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <type_traits>

namespace groundgrid {
namespace {
constexpr std::uint64_t basis=14695981039346656037ULL, prime=1099511628211ULL;
struct Binary {
    std::istream* in=nullptr;
    std::ostream* out=nullptr;
    std::uint64_t hash=basis;
    void bytes(void* p,std::size_t n,bool checksum=true) {
        auto* data=static_cast<unsigned char*>(p);
        if(in) in->read(reinterpret_cast<char*>(p),static_cast<std::streamsize>(n));
        else out->write(reinterpret_cast<const char*>(p),static_cast<std::streamsize>(n));
        if((in && !*in) || (out && !*out)) throw std::runtime_error("snapshot IO/truncation");
        if(checksum) for(std::size_t i=0;i<n;++i) { hash^=data[i]; hash*=prime; }
    }
    void u64(std::uint64_t& value,bool checksum=true) {
        unsigned char b[8];
        if(out) for(int i=0;i<8;++i) b[i]=static_cast<unsigned char>(value>>(8*i));
        bytes(b,8,checksum);
        if(in) { value=0; for(int i=0;i<8;++i) value|=std::uint64_t(b[i])<<(8*i); }
    }
    void number(double& value) {
        static_assert(sizeof(double)==8 && std::numeric_limits<double>::is_iec559,"IEEE doubles required");
        std::uint64_t bits=0;
        if(out) std::memcpy(&bits,&value,8);
        u64(bits);
        if(in) std::memcpy(&value,&bits,8);
    }
    void integer(int& value) {
        double d=value; number(d);
        if(in) {
            if(!std::isfinite(d) || d!=std::trunc(d) ||
               d<std::numeric_limits<int>::min() || d>std::numeric_limits<int>::max())
                throw std::runtime_error("invalid snapshot integer");
            value=static_cast<int>(d);
        }
    }
    void string(std::string& s) {
        std::uint64_t size=s.size(); u64(size);
        if(size>4096) throw std::runtime_error("snapshot string too long");
        if(in) s.resize(static_cast<std::size_t>(size));
        bytes(s.data(),s.size());
    }
    void floats(std::vector<float>& v) {
        static_assert(sizeof(float)==4 && std::numeric_limits<float>::is_iec559,"IEEE floats required");
        std::uint64_t size=v.size(); u64(size);
        if(size>4000000) throw std::runtime_error("snapshot array too large");
        if(in) v.resize(static_cast<std::size_t>(size));
        for(auto& x:v) {
            unsigned char b[4]; std::uint32_t bits=0;
            if(out) { std::memcpy(&bits,&x,4); for(int i=0;i<4;++i) b[i]=static_cast<unsigned char>(bits>>(8*i)); }
            bytes(b,4);
            if(in) { for(int i=0;i<4;++i) bits|=std::uint32_t(b[i])<<(8*i); std::memcpy(&x,&bits,4); }
        }
    }
    void pose(Pose2D& p) { number(p.x); number(p.y); number(p.yaw); }
};

void transfer(Binary& stream,PlanningInput& input) {
    std::string magic="groundgrid.planning-input.v1";
    stream.string(magic);
    if(magic!="groundgrid.planning-input.v1") throw std::runtime_error("unsupported planning snapshot version");
    stream.u64(input.attempt_id); stream.u64(input.goal_id); stream.u64(input.goal_stamp_ns);
    stream.u64(input.start_stamp_ns); stream.u64(input.map_stamp_ns);
    stream.string(input.frame); stream.string(input.source);
    stream.pose(input.start); stream.pose(input.goal);
    input.config.visit([&](const char* name,auto& value) {
        std::string key=name; stream.string(key);
        if(key!=name) throw std::runtime_error("snapshot configuration mismatch");
        double d=static_cast<double>(value); stream.number(d);
        if(!std::isfinite(d)) throw std::runtime_error("nonfinite snapshot configuration");
        using T=std::decay_t<decltype(value)>;
        if constexpr(std::is_integral_v<T>) {
            if(d!=std::trunc(d) || d<double(std::numeric_limits<T>::lowest()) ||
               d>double(std::numeric_limits<T>::max())) throw std::runtime_error("invalid configuration integer");
        }
        if(stream.in) value=static_cast<T>(d);
    });
    auto& m=input.map;
    stream.integer(m.rows); stream.integer(m.cols); stream.integer(m.start_row); stream.integer(m.start_col);
    stream.number(m.resolution); stream.number(m.center_x); stream.number(m.center_y);
    stream.number(m.length_x); stream.number(m.length_y);
    stream.floats(m.cost); stream.floats(m.gx); stream.floats(m.gy); stream.floats(m.slope);
    int bins=input.primitives.headingBins(); stream.integer(bins);
    if(bins<0 || bins>128) throw std::runtime_error("invalid primitive heading count");
    std::vector<MotionPrimitive> primitives;
    if(stream.out) for(int bin=0;bin<bins;++bin) {
        const auto& group=input.primitives.primitivesFor(bin);
        primitives.insert(primitives.end(),group.begin(),group.end());
    }
    std::uint64_t count=primitives.size(); stream.u64(count);
    if(count>32768) throw std::runtime_error("too many primitives");
    if(stream.in) primitives.resize(static_cast<std::size_t>(count));
    std::uint64_t samples=0;
    for(auto& p:primitives) {
        stream.integer(p.start_bin); stream.integer(p.end_bin); stream.integer(p.direction);
        stream.number(p.dx); stream.number(p.dy); stream.number(p.dyaw);
        stream.number(p.length); stream.number(p.base_cost);
        std::uint64_t size=p.samples.size(); stream.u64(size);
        if(size>2000000 || (samples+=size)>2000000) throw std::runtime_error("too many primitive samples");
        if(stream.in) { p.samples.resize(size); p.v_profile.resize(size); p.w_profile.resize(size); }
        if(p.v_profile.size()!=size || p.w_profile.size()!=size) throw std::runtime_error("primitive profile mismatch");
        for(std::size_t i=0;i<size;++i) {
            stream.pose(p.samples[i]); stream.number(p.v_profile[i]); stream.number(p.w_profile[i]);
        }
    }
    if(stream.in && !input.primitives.restore(bins,primitives)) throw std::runtime_error("invalid primitives");
    const auto expected=stream.hash;
    std::uint64_t actual=expected; stream.u64(actual,false);
    if(actual!=expected) throw std::runtime_error("planning snapshot checksum mismatch");
    if(!m.valid()) throw std::runtime_error("invalid planning grid");
}

std::string quote(const std::string& text) {
    std::ostringstream out; out<<'"';
    for(unsigned char c:text) {
        if(c=='"' || c=='\\') out<<'\\'<<c;
        else if(c<32) out<<"\\u"<<std::hex<<std::setw(4)<<std::setfill('0')<<unsigned(c)<<std::dec;
        else out<<c;
    }
    out<<'"'; return out.str();
}
void numberJson(std::ostream& out,double d) {
    if(std::isfinite(d)) out<<std::setprecision(17)<<d; else out<<"null";
}
void poseJson(std::ostream& out,const Pose2D& p) {
    out<<'['; numberJson(out,p.x); out<<','; numberJson(out,p.y); out<<','; numberJson(out,p.yaw); out<<']';
}
void writeText(const std::filesystem::path& file,const std::string& text) {
    std::ofstream out(file,std::ios::binary); out<<text<<'\n'; out.close();
    if(!out) throw std::runtime_error("snapshot metadata write failed");
}
} // namespace

void savePlanningSnapshot(const std::string& path,const PlanningInput& input) {
    if(std::filesystem::exists(path)) throw std::runtime_error("refusing to overwrite snapshot");
    // Writer-owned copy: transfer is symmetric. Never round the in-memory library via
    // its legacy nine-digit .dat writer or reopen the source file while ROS is running.
    PlanningInput copy=input;
    const std::string temporary=path+".partial";
    if(std::filesystem::exists(temporary)) throw std::runtime_error("snapshot partial already exists");
    std::ofstream out(temporary,std::ios::binary);
    Binary stream{nullptr,&out}; transfer(stream,copy);
    out.close(); if(!out) throw std::runtime_error("snapshot close failed");
    std::filesystem::rename(temporary,path);
}
PlanningInput loadPlanningSnapshot(const std::string& path) {
    if(std::filesystem::file_size(path)>256ULL*1024*1024) throw std::runtime_error("snapshot file too large");
    std::ifstream in(path,std::ios::binary);
    Binary stream{&in,nullptr}; PlanningInput input; transfer(stream,input);
    if(in.peek()!=std::char_traits<char>::eof()) throw std::runtime_error("trailing snapshot data");
    return input;
}
std::string planningResultJson(const PlanningInput& input,const PlanningResult& r) {
    std::ostringstream out;
    out<<"{\"schema_version\":1,\"attempt_id\":"<<input.attempt_id<<",\"goal_id\":"<<input.goal_id
       <<",\"goal_stamp_ns\":"<<input.goal_stamp_ns<<",\"start_stamp_ns\":"<<input.start_stamp_ns
       <<",\"map_stamp_ns\":"<<input.map_stamp_ns<<",\"frame\":"<<quote(input.frame)
       <<",\"source\":"<<quote(input.source)<<",\"ok\":"<<(r.ok ? "true":"false")
       <<",\"reason\":"<<quote(r.reason)<<",\"snapped\":"<<(r.snapped ? "true":"false")
       <<",\"expanded\":"<<r.expanded<<",\"root_successors\":"<<r.root_successors;
    for(const auto& item:std::initializer_list<std::pair<const char*,double>>{
        {"total_ms",r.total_ms},{"snap_ms",r.snap_ms},{"search_ms",r.search_ms},{"profile_ms",r.profile_ms},
        {"snap_distance_m",r.snap_distance},{"path_length_m",r.path_length},
        {"reverse_length_m",r.reverse_length},{"route_cost",r.route_cost}}) {
        out<<','<<quote(item.first)<<':'; numberJson(out,item.second);
    }
    out<<",\"start\":"; poseJson(out,input.start);
    out<<",\"requested_goal\":"; poseJson(out,input.goal);
    out<<",\"selected_goal\":"; poseJson(out,r.selected_goal);
    out<<",\"poses\":"<<r.path.poses.size()<<'}';
    return out.str();
}

PlanningSnapshotWriter::~PlanningSnapshotWriter() { finish(); }
void PlanningSnapshotWriter::start(const std::string& directory) {
    if(directory.empty()) return;
    if(worker_.joinable() || stopping_) throw std::runtime_error("snapshot writer already configured");
    // A fresh root prevents attempt-id collisions after node restarts.
    if(!std::filesystem::create_directory(directory)) throw std::runtime_error("snapshot directory must be new");
    directory_=directory;
    worker_=std::thread(&PlanningSnapshotWriter::run,this);
}
bool PlanningSnapshotWriter::submit(PlanningInput input,PlanningResult result) {
    if(!enabled()) return false;
    std::lock_guard<std::mutex> lock(mutex_);
    ++stats_.submitted;
    if(stopping_ || queue_.size()>=4) { ++stats_.dropped; return false; }
    queue_.push_back({std::move(input),std::move(result)}); ready_.notify_one(); return true;
}
PlanningSnapshotWriter::Stats PlanningSnapshotWriter::stats() const {
    std::lock_guard<std::mutex> lock(mutex_); return stats_;
}
void PlanningSnapshotWriter::finish() {
    { std::lock_guard<std::mutex> lock(mutex_); stopping_=true; ready_.notify_all(); }
    if(worker_.joinable()) worker_.join();
}
void PlanningSnapshotWriter::run() {
    for(;;) {
        Item item;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            ready_.wait(lock,[&]{return stopping_ || !queue_.empty();});
            if(queue_.empty()) break;
            item=std::move(queue_.front()); queue_.pop_front();
        }
        try {
            const auto base=std::filesystem::path(directory_)/("attempt-"+std::to_string(item.input.attempt_id));
            savePlanningSnapshot(base.string()+".ggsnap",item.input);
            writeText(base.string()+".json",planningResultJson(item.input,item.result));
            std::lock_guard<std::mutex> lock(mutex_); ++stats_.written;
        } catch(const std::exception&) {
            std::lock_guard<std::mutex> lock(mutex_); ++stats_.failed;
        }
        // Even a terminated ROS run leaves a last-known summary. A missing or incomplete
        // summary is an archival failure, never an implicit successful recording.
        const auto s=stats();
        std::ostringstream summary;
        summary<<"{\"submitted\":"<<s.submitted<<",\"written\":"<<s.written
               <<",\"dropped\":"<<s.dropped<<",\"failed\":"<<s.failed<<'}';
        try { writeText(std::filesystem::path(directory_)/"writer-summary.json",summary.str()); }
        catch(const std::exception&) { std::lock_guard<std::mutex> lock(mutex_); ++stats_.failed; }
    }
}
} // namespace groundgrid
