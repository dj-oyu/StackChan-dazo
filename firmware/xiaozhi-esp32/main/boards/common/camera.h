#ifndef CAMERA_H
#define CAMERA_H

#include <string>
#include <functional>
#include <utility>

class Camera {
public:
    // Optional hook for "describe the captured frame". When set, Explain()
    // implementations should delegate to this instead of their built-in
    // explain_url HTTP path. This lets a host (e.g. the Grok realtime protocol)
    // plug in its own vision backend while the MCP camera tools stay the single
    // source of the capture/confirm flow. Returns true and fills `description`
    // on success; false to fall back to the built-in path.
    using ExplainDelegate = std::function<bool(const std::string& question, std::string& description)>;
    void SetExplainDelegate(ExplainDelegate delegate) { explain_delegate_ = std::move(delegate); }

    virtual void SetExplainUrl(const std::string& url, const std::string& token) = 0;
    virtual bool Capture() = 0;
    virtual bool EncodeToJpegDataUri(std::string& data_uri, int quality = 80) { return false; }
    virtual bool SetHMirror(bool enabled) = 0;
    virtual bool SetVFlip(bool enabled) = 0;
    virtual bool SetSwapBytes(bool enabled) { return false; }  // Optional, default no-op
    virtual std::string Explain(const std::string& question) = 0;

protected:
    ExplainDelegate explain_delegate_;
};

#endif // CAMERA_H
