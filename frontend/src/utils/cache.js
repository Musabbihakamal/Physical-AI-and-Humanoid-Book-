// frontend/src/utils/cache.js
class FrontendCache {
  constructor() {
    this.cache = new Map();
    this.maxSize = 100; // Maximum number of items to cache
    this.defaultTTL = 300000; // Default TTL: 5 minutes in milliseconds
  }

  // Generate a cache key based on the parameters
  generateKey(type, params) {
    const paramString = JSON.stringify(params);
    return `${type}:${btoa(paramString)}`;
  }

  // Get an item from the cache
  get(key) {
    if (!this.cache.has(key)) {
      return null;
    }

    const item = this.cache.get(key);
    const now = Date.now();

    // Check if the item has expired
    if (now > item.expiry) {
      this.cache.delete(key);
      return null;
    }

    return item.value;
  }

  // Set an item in the cache
  set(key, value, ttl = null) {
    // If cache is at max size, remove oldest item
    if (this.cache.size >= this.maxSize) {
      const firstKey = this.cache.keys().next().value;
      this.cache.delete(firstKey);
    }

    const expiry = Date.now() + (ttl || this.defaultTTL);
    this.cache.set(key, { value, expiry });
  }

  // Delete an item from the cache
  delete(key) {
    return this.cache.delete(key);
  }

  // Clear the entire cache
  clear() {
    this.cache.clear();
  }

  // Get cache statistics
  stats() {
    return {
      size: this.cache.size,
      maxSize: this.maxSize,
      defaultTTL: this.defaultTTL
    };
  }
}

// Create a global cache instance
const cache = new FrontendCache();

// Export the cache instance and utility functions
export default cache;

// Export a function to cache API responses
export const cacheApiCall = async (cacheKey, apiCall, ttl = null) => {
  // Try to get from cache first
  const cachedResult = cache.get(cacheKey);
  if (cachedResult) {
    return { data: cachedResult, fromCache: true };
  }

  // If not in cache, make the API call
  const result = await apiCall();

  // Store in cache
  cache.set(cacheKey, result, ttl);

  return { data: result, fromCache: false };
};