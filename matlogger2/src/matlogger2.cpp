#include <matlogger2/matlogger2.h>
#include <iostream>
#include <boost/algorithm/string.hpp>

#include "thread.h"
#include "matlogger2_backend.h"


using namespace XBot::matlogger2;

using namespace XBot;

namespace
{
    template <typename Func>
    double measure_sec(Func f)
    {
        auto tic = std::chrono::high_resolution_clock::now();
        
        f();
        
        auto toc = std::chrono::high_resolution_clock::now();
        
        return std::chrono::duration_cast<std::chrono::nanoseconds>(toc-tic).count()*1e-9;
    }
}

class MatLogger2::MutexImpl
{
public:
    
    matlogger2::MutexType& get()
    {
        return _mutex;
    }
    
private:
    
    matlogger2::MutexType _mutex;
};

const std::string& VariableBuffer::get_name() const
{
    return _name;
}

XBot::MatLogger2::Options::Options():
    enable_compression(false),
    default_buffer_size(1e4)
{
}


namespace{
    
    std::string get_file_extension(std::string file)
    {
        std::vector<std::string> token_list;
        boost::split(token_list, file, [](char c){return c == '.';});
        
        if(token_list.size() > 1)
        {
            return token_list.back();
        }
        
        return "";
    }
    
    std::string date_time_as_string()
    {
        time_t rawtime;
        struct tm * timeinfo;
        char buffer [80];
        memset(buffer, 0, 80*sizeof(buffer[0]));

        std::time(&rawtime);
        timeinfo = localtime(&rawtime);

        strftime(buffer, 80, "%Y_%m_%d__%H_%M_%S", timeinfo);
        
        return std::string(buffer);
    }
    
}

MatLogger2::MatLogger2(std::string file, Options opt):
    _file_name(file),
    _vars_mutex(new MutexImpl),
    _buffer_mode(VariableBuffer::Mode::producer_consumer),
    _opt(opt)
{
    // get the file extension, or empty string if there is none
    std::string extension = get_file_extension(file);
    
    if(extension == "") // no extension, append date/time + .mat
    {
        _file_name += "__" + date_time_as_string() + ".mat";
    }
    else if(extension != "mat") // extension different from .mat, error
    {
        throw std::invalid_argument("MAT-file name should either have .mat \
extension, or no extension at all");
    }
    
    // create mat file (erase if existing already)
    _backend = Backend::MakeInstance("matio");
    
    if(!_backend)
    {
        throw std::runtime_error("MatLogger2: unable to create backend");
    }
    
    if(!_backend->init(_file_name, _opt.enable_compression))
    {
        std::cerr << "\033[0;31m----------------------------------------------------------" << std::endl;
        std::cerr << "| ERROR: most likely you did not create the /log folder! |" << std::endl;
        std::cerr << "----------------------------------------------------------\033[0m" << std::endl;
        throw std::runtime_error("MatLogger2: unable to initialize backend");
    }
}

const std::string& MatLogger2::get_filename() const
{
    return _file_name;
}

MatLogger2::Options MatLogger2::get_options() const
{
    return _opt;
}

void MatLogger2::set_on_data_available_callback(VariableBuffer::CallbackType callback)
{
    std::lock_guard<MutexType> lock(_vars_mutex->get());    
    
    for(auto& p : _vars)
    {
        p.second.set_on_block_available(callback);
    }
    
    _on_block_available = callback;
}

void XBot::MatLogger2::set_buffer_mode(VariableBuffer::Mode buffer_mode)
{
    std::lock_guard<MutexType> lock(_vars_mutex->get());    
    
    for(auto& p : _vars)
    {
        p.second.set_buffer_mode(buffer_mode);
    }
    
    _buffer_mode = buffer_mode;
}


bool MatLogger2::create(const std::string& var_name, int rows, int cols, int buffer_size)
{
    if(buffer_size == -1)
    {
        buffer_size = _opt.default_buffer_size;
    }
    
    if(!(rows > 0 && cols > 0 && buffer_size > 0))
    {
        fprintf(stderr, "Unable to create variable '%s': invalid parameters \
(rows=%d, cols=%d, buf_size=%d)\n",
                var_name.c_str(), rows, cols, buffer_size);
        return false;
    }
    
    std::lock_guard<MutexType> lock(_vars_mutex->get());    
    
    // check if variable is already defined (in which case, return false)
    auto it = _vars.find(var_name);
    
    if(it != _vars.end())
    {
        fprintf(stderr, "Variable '%s' already exists\n", var_name.c_str());
        return false;
    }
    
    // compute block size from required buffer_size and number of blocks in
    // queue
    int block_size = std::max(1, buffer_size / VariableBuffer::NumBlocks());
    
    printf("Created variable '%s' (%d blocks, %d elem each)\n", 
           var_name.c_str(), VariableBuffer::NumBlocks(), block_size);
    
    // insert VariableBuffer object inside the _vars map
    _vars.emplace(std::piecewise_construct,
                  std::forward_as_tuple(var_name),
                  std::forward_as_tuple(var_name, rows, cols, block_size));
    
    // set callback: this will be called whenever a new data block is 
    // available in the variable queue
    _vars.at(var_name).set_on_block_available(_on_block_available);
    _vars.at(var_name).set_buffer_mode(_buffer_mode);
    
    return true;
}

bool MatLogger2::add(const std::string& var_name, double scalar)
{
    // turn scalar into a 1x1 matrix
    Eigen::Matrix<double, 1, 1> data(scalar);
    
    VariableBuffer * vbuf = find_or_create(var_name, data.rows(), data.cols());
    
    return vbuf && vbuf->add_elem(data);
}


int MatLogger2::flush_available_data()
{
    // number of flushed bytes is returned on exit
    int bytes = 0;
    
    // acquire exclusive access to the _vars object
    std::lock_guard<MutexType> lock(_vars_mutex->get());    
    for(auto& p : _vars)
    {
        Eigen::MatrixXd block;
        int valid_elems = 0;
        
        // while there are blocks available for reading..
        while(p.second.read_block(block, valid_elems))
        {
            // get rows & cols for a single sample
            auto dims = p.second.get_dimension();
            
            int rows = -1;
            int cols = -1;
            int slices = -1;
            bool is_vector = false;
            
            
            if(dims.second == 1) // the variable is a vector
            {
                rows = dims.first;
                cols = valid_elems;
                slices = 1;
                is_vector = true;
            }
            else // the variable is a matrix
            {
                rows = dims.first;
                cols = dims.second;
                slices = valid_elems;
                is_vector = false;
            }
        
            // write block
            _backend->write(p.second.get_name().c_str(),
                            block.data(),
                            rows, cols, slices);
            
            // update bytes computation
            bytes += block.rows() * valid_elems * sizeof(double);
        }
    }
    
    return bytes;
}

XBot::VariableBuffer * XBot::MatLogger2::find_or_create(const std::string& var_name, 
                                                        int rows, int cols)
{
    // try to find var_name
    auto it = _vars.find(var_name);
    
    // if we found it, return a valid pointer
    if(it != _vars.end())
    {
        return &(it->second);
    }
    
    // if it was not found, and we cannot create it, return nullptr
    if(!create(var_name, rows, cols))
    {
         return nullptr;
    }
    
    // we managed to create the variable, try again to find it
    return find_or_create(var_name, rows, cols);
    
}


bool MatLogger2::flush_to_queue_all()
{
    bool ret = true;
    for(auto& p : _vars)
    {
        ret = p.second.flush_to_queue() && ret;
    }
    return ret;
}


MatLogger2::~MatLogger2()
{
    /* inside this constructor, we have the guarantee that the flusher thread
    * (if running) is not using this object
    */
    
    // de-register any callback
    set_on_data_available_callback(VariableBuffer::CallbackType());
    
    // set producer_consumer mode to be able to call read_block()
    set_buffer_mode(VariableBuffer::Mode::producer_consumer);
    
    // flush to queue and then flush to disk till all buffers are empty
    while(!flush_to_queue_all())
    {
        flush_available_data();
    }
    
    // flush to disk remaining data from queues
    while(flush_available_data() > 0);
    
    printf("Flushed all data for file '%s'\n", _file_name.c_str());
    
    _backend->close();
}



#define ADD_EXPLICIT_INSTANTIATION(EigenType) \
template bool MatLogger2::add(const std::string&, const Eigen::MatrixBase<EigenType>&); \
template bool VariableBuffer::add_elem(const Eigen::MatrixBase<EigenType>&); \
template bool VariableBuffer::BufferBlock::add(const Eigen::MatrixBase<EigenType>&);

#define ADD_EXPLICIT_INSTANTIATION_STD_VECTOR(Scalar) \
template bool MatLogger2::add(const std::string&, const std::vector<Scalar>&); \


template <typename Scalar>
using Vector6 = Eigen::Matrix<Scalar,6,1>;

template <typename Scalar>
using MapX = Eigen::Map<Eigen::Matrix<Scalar, -1, 1>>;

ADD_EXPLICIT_INSTANTIATION(Eigen::MatrixXd)
ADD_EXPLICIT_INSTANTIATION(Eigen::Matrix2d)
ADD_EXPLICIT_INSTANTIATION(Eigen::Matrix3d)
ADD_EXPLICIT_INSTANTIATION(Eigen::Matrix4d)

ADD_EXPLICIT_INSTANTIATION(Eigen::MatrixXf)
ADD_EXPLICIT_INSTANTIATION(Eigen::Matrix2f)
ADD_EXPLICIT_INSTANTIATION(Eigen::Matrix3f)
ADD_EXPLICIT_INSTANTIATION(Eigen::Matrix4f)

ADD_EXPLICIT_INSTANTIATION(Eigen::VectorXd)
ADD_EXPLICIT_INSTANTIATION(Eigen::Vector2d)
ADD_EXPLICIT_INSTANTIATION(Eigen::Vector3d)
ADD_EXPLICIT_INSTANTIATION(Eigen::Vector4d)
ADD_EXPLICIT_INSTANTIATION(Vector6<double>)

ADD_EXPLICIT_INSTANTIATION(Eigen::VectorXf)
ADD_EXPLICIT_INSTANTIATION(Eigen::Vector2f)
ADD_EXPLICIT_INSTANTIATION(Eigen::Vector3f)
ADD_EXPLICIT_INSTANTIATION(Eigen::Vector4f)
ADD_EXPLICIT_INSTANTIATION(Vector6<float>)

ADD_EXPLICIT_INSTANTIATION(Eigen::VectorXi)
ADD_EXPLICIT_INSTANTIATION(Eigen::Vector2i)
ADD_EXPLICIT_INSTANTIATION(Eigen::Vector3i)
ADD_EXPLICIT_INSTANTIATION(Eigen::Vector4i)
ADD_EXPLICIT_INSTANTIATION(Vector6<int>)

ADD_EXPLICIT_INSTANTIATION_STD_VECTOR(int)
ADD_EXPLICIT_INSTANTIATION_STD_VECTOR(unsigned int)
ADD_EXPLICIT_INSTANTIATION_STD_VECTOR(float)
ADD_EXPLICIT_INSTANTIATION_STD_VECTOR(double)
