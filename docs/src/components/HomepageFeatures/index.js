import React from "react";
import clsx from "clsx";
import styles from "./styles.module.css";
import linkedInSvg from "../../../static/img/linkedin.png";

const FeatureList = [
  {
    name: "Elisa Flemes",
    Svg: require("@site/static/img/undraw_docusaurus_mountain.svg").default,
    link: (
      <>
        Docusaurus was designed from the ground up to be easily installed and
        used to get your website up and running quickly.
      </>
    ),
  },
  {
    name: "Felipe Campos",
    Svg: require("@site/static/img/undraw_docusaurus_tree.svg").default,
    link: (
      <>
        Docusaurus lets you focus on your docs, and we&apos;ll do the chores. Go
        ahead and move your docs into the <code>docs</code> directory.
      </>
    ),
  },
  {
    name: "Gabriela Barreto",
    Svg: require("@site/static/img/undraw_docusaurus_react.svg").default,
    link: (
      <>
        Extend or customize your website layout by reusing React. Docusaurus can
        be extended while reusing the same header and footer.
      </>
    ),
  },
  {
    name: "Gustavo Ferreira",
    Svg: require("@site/static/img/undraw_docusaurus_react.svg").default,
    link: (
      <>
        Extend or customize your website layout by reusing React. Docusaurus can
        be extended while reusing the same header and footer.
      </>
    ),
  },
  {
    name: "Henrique Lemos",
    Svg: require("@site/static/img/undraw_docusaurus_react.svg").default,
    link: (
      <>
        Extend or customize your website layout by reusing React. Docusaurus can
        be extended while reusing the same header and footer.
      </>
    ),
  },
  {
    name: "Henrique Marlon",
    Svg: require("@site/static/img/undraw_docusaurus_react.svg").default,
    link: (
      <>
        Extend or customize your website layout by reusing React. Docusaurus can
        be extended while reusing the same header and footer.
      </>
    ),
  },
  {
    name: "Paulo Evangelista",
    Svg: require("@site/static/img/undraw_docusaurus_react.svg").default,
    link: (
      <>
        Extend or customize your website layout by reusing React. Docusaurus can
        be extended while reusing the same header and footer.
      </>
    ),
  },
];

function Feature({ Svg, name, link }) {
  return (
    <div
      style={{
        background: "#EEF7FF",
        margin: "10px",
        borderRadius: "5px",
        width: "30%",
        margin: "1.5%",
      }}
    >
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <h3 style={{color: '#79A4FA'}}>{name}</h3>
        <a style={{textDecoration: 'underline', display: 'flex', alignItems:'center'}} href={link}>  <img
            src={linkedInSvg}
            alt="LinkedIn"
            className={styles.linkedInSvg}
            style={{width: '30px', height: '30px', marginRight: '5px'}}
          />
          LinkedIn</a>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="text--center">
          <h1>Get to know the group members</h1>
        </div>

        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
