import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  Svg: React.ComponentType<React.ComponentProps<'svg'>>;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Module 1: ROS 2 Fundamentals',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default,
    description: (
      <>
        Master the Robot Operating System 2 middleware, build Python agents,
        and model humanoid robots with URDF. The foundation for all robotics development.
      </>
    ),
  },
  {
    title: 'Module 2: The Digital Twin',
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default,
    description: (
      <>
        Simulate robots in <strong>Gazebo</strong> with realistic physics, render in <strong>Unity</strong>
        for photorealism, and configure <strong>sensors</strong> (LiDAR, cameras, IMU) with accurate noise models.
      </>
    ),
  },
  {
    title: 'Hands-On Learning',
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default,
    description: (
      <>
        Every chapter includes runnable examples, downloadable code, practice exercises,
        and troubleshooting guides. Build real robots, not just theory.
      </>
    ),
  },
];

function Feature({title, Svg, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
