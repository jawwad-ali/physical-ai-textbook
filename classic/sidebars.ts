import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: ROS 2',
      items: [
        'tutorial-basics/congratulations',
        // Add your module 1 docs here
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Gazebo & Unity',
      items: [
        'tutorial-extras/manage-docs-versions',
        // Add your module 2 docs here
      ],
    },
  ],
};

export default sidebars;