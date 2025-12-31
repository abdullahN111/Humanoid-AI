# Quickstart: ROS 2 as a Robotic Nervous System Documentation

## Overview
This quickstart guide will help you set up and run the ROS 2 documentation module on your local machine.

## Prerequisites
- Node.js (version 18 or higher)
- npm or yarn package manager
- Git for version control
- Basic knowledge of ROS 2 concepts (helpful but not required)

## Setup Instructions

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Install Dependencies
```bash
cd website  # Navigate to the Docusaurus project directory
npm install
```

### 3. Start the Development Server
```bash
npm start
```

This command starts a local development server and opens the documentation in your browser at `http://localhost:3000`.

### 4. Verify Documentation Structure
After the server starts, you should see:
- Navigation menu with "Module 1: ROS 2 as a Robotic Nervous System"
- Three chapters under the module: "Introduction", "Python AI Agents", and "URDF Structure"
- Proper styling and layout consistent with Docusaurus standards

## Running Tests (if applicable)
```bash
npm test
```

## Building for Production
```bash
npm run build
```

This command creates an optimized build of the documentation site in the `build/` directory.

## Key Directories and Files
- `docs/module-1/` - Contains the ROS 2 documentation content
- `docs/module-1/chapter-1.md` - Introduction to ROS 2 as a nervous system
- `docs/module-1/chapter-2.md` - Connecting Python AI agents to ROS 2
- `docs/module-1/chapter-3.md` - Humanoid structure with URDF
- `docusaurus.config.js` - Main configuration for the site
- `sidebars.js` - Navigation structure

## Troubleshooting
- If you encounter issues with dependencies, try clearing the cache: `npm cache clean --force` then reinstall
- If the site doesn't load properly, check the console for error messages
- Make sure you're running Node.js version 18 or higher: `node --version`

## Next Steps
1. Review the documentation content in the `docs/` directory
2. Customize the content to match your specific ROS 2 setup
3. Add additional examples or tutorials as needed
4. Deploy the documentation to your preferred hosting platform