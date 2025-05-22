// === vulkan_processor.hpp ===
#include <vector>
#include <cstdint>
#include <opencv2/opencv.hpp>
#include <vulkan/vulkan.h>

class VulkanProcessor {
    public:
        VulkanProcessor();
        ~VulkanProcessor();
    
        bool initialize();
        void processImage(const cv::Mat& image);
    
    private:
        // Vulkan handles
        VkInstance instance_ = VK_NULL_HANDLE;
        VkPhysicalDevice physicalDevice_ = VK_NULL_HANDLE;
        VkDevice device_ = VK_NULL_HANDLE;
        VkQueue computeQueue_ = VK_NULL_HANDLE;
        uint32_t computeQueueFamilyIndex_ = 0;
        VkCommandPool commandPool_ = VK_NULL_HANDLE;
    
        // Core steps
        bool createInstance();
        bool pickPhysicalDevice();
        bool createLogicalDevice();
        bool createCommandPool();
        void cleanup();
    };
    