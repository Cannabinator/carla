# 🌐 Windows Access Guide

## Problem: Cannot Access Frontend from Windows

### Your Network Setup
- **Ubuntu Machine**: 192.168.1.113 (running web server)
- **Windows Machine**: 192.168.1.110 (CARLA server)
- **Web Server Port**: 8000

## ✅ Quick Start

### Option 1: Standalone Web Viewer (Recommended)
```bash
# On Ubuntu machine
./start_web_viewer.sh
```

Then on Windows, open browser to:
```
http://192.168.1.113:8000
```

### Option 2: With CARLA Scenario
```bash
# On Ubuntu machine
./run_v2v_lidar.sh
```

Then on Windows, open browser to:
```
http://192.168.1.113:8000
```

## 🔧 Troubleshooting

### 1. Check Server is Running
On Ubuntu:
```bash
ss -tlnp | grep :8000
# Should show: LISTEN on 0.0.0.0:8000
```

### 2. Test Local Access First
On Ubuntu:
```bash
curl http://localhost:8000
# Should return HTML content
```

### 3. Check Ubuntu Firewall
```bash
sudo ufw status
# If active, allow port 8000:
sudo ufw allow 8000/tcp
```

### 4. Check Windows Firewall
On Windows:
1. Open Windows Defender Firewall
2. Click "Advanced settings"
3. Check if port 8000 is blocked
4. Try turning off firewall temporarily to test

### 5. Verify Network Connectivity
From Windows Command Prompt:
```cmd
ping 192.168.1.113
# Should respond successfully

telnet 192.168.1.113 8000
# Should connect (if server is running)
```

### 6. Check Server is Binding to All Interfaces
The server should start with:
```
INFO: Uvicorn running on http://0.0.0.0:8000
```

Not:
```
INFO: Uvicorn running on http://127.0.0.1:8000  ❌ WRONG
```

## 🎯 Expected Output

When starting the server, you should see:

```
╔════════════════════════════════════════════════════════════════╗
║       V2V LiDAR Web Viewer - Standalone Mode                 ║
╚════════════════════════════════════════════════════════════════╝

📡 Server URLs:
   Local (Ubuntu):    http://localhost:8000
   Network (Windows): http://192.168.1.113:8000

💡 Access from Windows:
   1. Open browser on Windows
   2. Navigate to: http://192.168.1.113:8000
   3. Make sure Windows Firewall allows the connection
```

## 🔍 Common Issues

| Issue | Solution |
|-------|----------|
| Connection refused | Server not running - start with `./start_web_viewer.sh` |
| Connection timeout | Firewall blocking - check both Ubuntu and Windows firewalls |
| Port already in use | Another service using port 8000 - change to 8001 with `--port 8001` |
| Wrong IP address | Run `hostname -I` on Ubuntu to get correct IP |

## 📱 Alternative Ports

If port 8000 doesn't work, try:

```bash
# Ubuntu
python src/visualization/web/server.py --port 8001

# Windows browser
http://192.168.1.113:8001
```

## 🚀 Start Commands

### Standalone Web Viewer (No CARLA)
```bash
./start_web_viewer.sh
```

### Full V2V LiDAR Scenario
```bash
./run_v2v_lidar.sh
```

### Custom Port
```bash
python src/visualization/web/server.py --host 0.0.0.0 --port 8001
```

## ✅ Success Checklist

- [ ] Server shows "running on http://0.0.0.0:8000"
- [ ] `ss -tlnp | grep :8000` shows LISTEN
- [ ] `curl http://localhost:8000` returns HTML
- [ ] Windows can ping 192.168.1.113
- [ ] Ubuntu firewall allows port 8000
- [ ] Windows firewall allows outbound to port 8000
- [ ] Browser on Windows can access http://192.168.1.113:8000

## 📞 Still Not Working?

1. **Check exact error** in Windows browser
2. **Verify IP**: Run `hostname -I` on Ubuntu
3. **Test with different port**: `--port 8001`
4. **Disable firewalls** temporarily to isolate issue
5. **Check logs**: Look at server output for errors
