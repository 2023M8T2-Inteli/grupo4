import React from "react";
import { FaArrowRight } from "react-icons/fa6";
import clsx from "clsx";
import styles from "./styles.module.css";
import linkedInSvg from "../../../static/img/linkedin.png";

const FeatureList = [
  {
    course: "Engenharia de Computação",
    name: "Elisa Flemer",

    link: "https://www.linkedin.com/in/elisaflemer",
  },
  {
    course: "Engenharia de Computação",
    name: "Felipe Campos",
    link: "http://www.linkedin.com/in/felipe-pereira-campos-250aa2231",
  },
  {
    course: "Engenharia de Computação",
    name: "Gabriela Barreto",
    link: "https://www.linkedin.com/in/gabriela-barretto-dados",
  },
  {
    course: "Engenharia de Computação",
    name: "Gustavo Ferreira",
    link: "https://www.linkedin.com/in/gustavo-ferreira-oliveira/",
  },
  {
    course: "Engenharia de Computação",
    name: "Henrique Lemos",
    link: "https://www.linkedin.com/in/henriquelfmatias/",
  },
  {
    course: "Engenharia de Computação",
    name: "Henrique Marlon",
    link: "https://www.linkedin.com/in/henriquemarlon/",
  },
  {
    course: "Engenharia de Computação",
    name: "Paulo Evangelista",
    link: "https://www.linkedin.com/in/paulo-evangelista/",
  },
];

function Feature({ course, name, link }) {
  return (
    <a href={link} target="_blank" className={styles.card_container}>
      <div className="padding-horiz--md">
        <h4 className={styles.course}>{course}</h4>
        <h2 style={{ fontWeight: "300" }}>{name}</h2>
        <div className={styles.flex}>
          <img
            src={linkedInSvg}
            alt="LinkedIn"
            className={styles.linkedInSvg}
            style={{ width: "22px", height: "22px", marginRight: "16px" }}
          />
          <div className={`${styles.flex} ${styles.text}`}>
            LinkedIn
            <FaArrowRight style={{ marginLeft: "4px" }} size={16} />
          </div>

        </div>
      </div>
    </a>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="text--center">
          <h1>Conheça os integrantes do grupo 04</h1>
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
